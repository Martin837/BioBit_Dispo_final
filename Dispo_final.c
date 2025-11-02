#pragma GCC optimize ("O0")
#include <stdio.h>
#include "pico/stdlib.h"
#include "sd_card.h"
#include "ff.h"
#include "hardware/i2c.h"
#include "lib/ads.h"
#include "lib/MC3479.h"
#include "lib/Max.h"
#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include <string.h>
#include "hardware/flash.h"
#include "pico/flash.h"
#include "hardware/adc.h"
#include "pico/sleep.h"
#include "lib/max30100_spo2.h"
#include "hardware/powman.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"
#include "hardware/sync.h"

#define debug 0
static powman_power_state off_state;
static powman_power_state on_state;

//TODO check red comments and other TODO's
//TODO Test wifi, fall detection, and panic button

//Flash
extern uint32_t ADDR_PERSISTENT;
#define ADDR_PERSISTENT_BASE_ADDR ((uint32_t)&ADDR_PERSISTENT - XIP_BASE)
uintptr_t persistent_addr = (uintptr_t)&ADDR_PERSISTENT;

//wifi
#define WIFI_SSID "Labyrinth" //TODO receive credentials over uart and save them to flash
#define WIFI_PASS "alpha_november_tango_oscar"
#define SERVER_IP "192.168.0.100"  // Replace with server's IP
#define SERVER_PORT 4242

#define BUF_SIZE 2048
uint8_t sent = 0;
uint8_t data[2048] = {0}; //Wifi

static struct tcp_pcb *client_pcb = NULL;
//wifi

void get_adc_sample();
void save_ecg();
void save_spo();
void exec_flash_range_erase(void* param);
void exec_flash_range_program(void* param);
int disp_init();

static int powman_sleep(uint32_t secs);
void powman_init(uint64_t abs_time_ms);

//Intervals
uint16_t electro=0,spo=0;
uint16_t samples = 0;

//ECG
uint16_t ecg[1280] = {0};
bool ecg_done = false, ecg_saved = false;

//Battery
float bat_val = 0; 
const float bat_min_lvl = 3.4;

uint64_t last_bat_rd = 0;
const uint64_t bat_rd_t = 5 * 60e6;
uint8_t bat_read = 0;

//Funciones wifi //TODO change this to use lib functions
static err_t recv_callback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    if (!p) {
        printf("Disconnected from server.\n");
        tcp_close(tpcb);
        client_pcb = NULL;
        return ERR_OK;
    }
    printf("[Server]: %.*s\n", p->len, (char *)p->payload);

    char rec_data[BUF_SIZE] = {0};

    memcpy(rec_data, (char *)p->payload, strlen((char *)p->payload));

    //TODO receive data?
    
    tcp_recved(tpcb, p->len);
    pbuf_free(p);
    return ERR_OK;
}

static err_t connected_callback(void *arg, struct tcp_pcb *tpcb, err_t err) {
    if (err != ERR_OK) {
        printf("Connection failed: %d\n", err);
        return err;
    }
    printf("Connected to server!\n");
    client_pcb = tpcb;
    tcp_recv(tpcb, recv_callback);
    return ERR_OK;
}

err_t send_to_server(const char *msg) {
    if (client_pcb) {
        tcp_write(client_pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
        return tcp_output(client_pcb);
    }
}
//Funciones wifi


#define UART_ID uart0
#define BAUD_RATE 115200

// We are using pins 0 and 1, but see the GPIO function select table in the
// datasheet for information on which other pins can be used.
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define button 2

//i2c
#define I2C_PORT i2c0
#define I2C_SDA 4
#define I2C_SCL 5

bool config = false;

uint8_t indice = 0,recibo[4];
char caracter_rec;

struct ADS adsensor;
const uint8_t addr = 0x1F;
uint8_t electro_habi = 0, acel_habi = 0, max_habi = 0, cayo = 0, panico = 0, datalog = 0, wifi_hab = 0, first_run = 0;
bool mode_cont = false;
uint16_t ch0;
uint64_t t = 10e6, last_t = 0, samp_t = 0;

MAX30100_t max; //Sensor struct
float spo2 = 0;
bool max_done = false, max_saved = false;
spo2_filter_t spo2f;
uint16_t ir[500] = {0}, red[500] = {0};
uint16_t max_samp = 0;
uint64_t last_max_t = 0, last_spo2t = 0;
// callback to update LED currents
void set_led_current(uint8_t ir, uint8_t red) {
    MAX30100_cfg(&max, 0x03, 0x03, 0x01, ir, red, true);
}

void Recibe_car(); 
void setup();
static void sleep(uint32_t secs);

static void sleep_callback(void) {
    disp_init();
}

FRESULT fr;
FATFS fs;
FIL fil;
int ret;
char buf[100];
char filename_ecg[] = "ecg.csv";
char filename_spo[] = "spo2.csv";

//* Button
uint8_t state = 0, last_state = 0, panic_pressed = 0;
uint64_t pressed_time_i = 0, pressed_time = 0;

int main(){
    stdio_init_all();
    

    gpio_init(button);
    gpio_set_dir(button, 0);

    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }


    if(gpio_get(button)){
        config = true;
        setup();
    }

    disp_init();

    powman_init(1704067200000);

    while (true) {

        t = timer0_hw->timehr<<32 | timer0_hw->timelr;

        state = gpio_get(button);
        if(state != last_state){
            last_state = state;
            if(state){
                pressed_time_i = t;
            }
            else{
                pressed_time = t - pressed_time_i;
                
                if(pressed_time >= 5e6)
                    panic_pressed = 1;
                else
                    panic_pressed = 0;
            }
        }

        if(t > (last_bat_rd + bat_rd_t) && bat_read){ 
            adc_init();
            adc_gpio_init(26);
            adc_select_input(0);
            adc_hw->cs |= ADC_CS_START_ONCE_BITS;
            bat_read = 0;

            last_bat_rd = t;
        }

        if(adc_hw->cs & ADC_CS_READY_BITS){
            bat_val = ((adc_hw->result * 2)*3.3)/4096;
            bat_read = 1;
        }

        if(bat_val <= bat_min_lvl){
            printf("Low battery");//TODO Shutdown
        }

        if(acel_habi){
            cayo = 0;
            MC34X9_acc_t a = readRawAccel();
            // Raw counts
            printf("RAW X:%6d Y:%6d Z:%6d\n", a.XAxis, a.YAxis, a.ZAxis);
            // In g
            printf("G   X:%0.3f Y:%0.3f Z:%0.3f\n", a.XAxis_g, a.YAxis_g, a.ZAxis_g);

            if(a.XAxis>60 && a.YAxis>60 && a.ZAxis>60){
                cayo= 1;
                printf("se cayo");
            }
        }

        //* take ecg
        if(electro_habi && ((t/1e6)>((last_t/1e6)+electro) || first_run)){
            ecg_done = 0; //! Test this, idk if it'll work
            get_adc_sample();
        }
        if(ecg_done){
            samples = 0;
            memset(ecg, 0, sizeof(ecg));
        }

        //* take spo2
        if(max_habi && (t/1e6)>((last_spo2t/1e6)+spo)){
            max_saved = false;
            max_done = false;
        }

        if(max_habi){
            if(MAX30100_read(&max)){

                MAX30100_FIFO_CLR(&max);

                if(t > (last_max_t + 20e3)){
                    red[max_samp++] = max.ir;
                    ir[max_samp++] = max.red;
                }

                if(max_samp == 500 && !max_done){
                    spo2 = spo2_process_batch(&spo2f, ir, red, 500);
                    spo2_adjust_led_current(&spo2f, set_led_current);

                    if (spo2f.spo2_valid) {
                        printf("SpO2: %.2f%% | PI=%.4f | Quality=%.2f | IRcur=%u | REDcur=%u\n",
                            spo2, spo2f.last_pi, spo2f.signal_quality,
                            spo2f.ir_current, spo2f.red_current);
                        max_done = true;
                    } 
                    else {
                        printf("SpO2: -- (no finger or unstable signal)\n");
                        max_samp = 0;
                    }
                }
            }

            if(datalog && max_done && !max_saved){
                save_spo();
                max_samp = 0;
                last_spo2t = t;
            }
        }

        if(wifi_hab){
            cyw43_arch_poll();
            if ((client_pcb && max_done && ecg_done)||(client_pcb && panic_pressed)) { //! this should be data ready flag or smth
                
                //* wifi trama recibo
                //* !ooooooo-bbbbbbb-p-c-eeeeeeee(x1280)?
                //? oxigeno-bateria-panico-caida-electro

                
                data[0] = (uint8_t)spo2;
                data[1] = (uint8_t)(((bat_val - bat_min_lvl) * 100)/(bat_val-bat_min_lvl)); //! Test this
                data[2] = panic_pressed;
                data[3] = cayo;

                uint8_t split_data[2*1280] = {0};

                for(int i = 0; i < 1280; i++){ //! Test this
                    // Extract the high 8 bits
                    split_data[i * 2] = (uint8_t)(ecg[i] >> 8); 
                    // Extract the low 8 bits
                    split_data[i * 2 + 1] = (uint8_t)(ecg[i] & 0xFF);
                }

                memcpy(data+4, split_data, strlen(split_data));

                if(send_to_server(data) == ERR_OK)
                    sent = 1;
            }
        }

        if(((wifi_hab && sent) || (datalog && ecg_saved && max_saved) || (!wifi_hab && !datalog)) && !mode_cont && !state){
            ads_stop(&adsensor);
            MAX30100_cfg(&max, 0x80, 0, 0, 0, 0, 0);
            stop();
            printf("Going to sleep, good night! \r\n");
            powman_sleep((electro > spo) ? electro : spo);
            sleep_power_up();
            printf("*yawns* Good morning!");
            cyw43_arch_init();
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        }
    }
}

void get_adc_sample(){
    if(t > samp_t+7812){
        bool read_suc = ads_read_channel0(&adsensor, &ch0);
        if(read_suc){
            samp_t = t;
            samples++;
            ecg[samples] = ch0;
            //printf("Read sample %d \n", samples);
        }
        else{
            samp_t = t;
            samples = 0;
        }
    }
    if(samples == 1280){
        last_t = t;
        first_run = 0;
        ecg_done = true;
        if(datalog) 
            save_ecg();
    }
}

void save_spo(){
    // Open file for writing ()
    fr = f_open(&fil, filename_spo, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
    }

    ret = f_printf(&fil, "%d,\n", (uint16_t)spo2);
    if (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
    }
    else{
        printf("Saved SPO2\n");
        max_saved = true;
    }

    // Close file
    fr = f_close(&fil);
}

void save_ecg(){
    // Open file for writing ()
    fr = f_open(&fil, filename_ecg, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
    }

    uint16_t written = 0;

    for(int i = 0; i < 1280; i++){
        ret = f_printf(&fil, "%d,\n", ecg[i]);
        if (ret < 0) {
            printf("ERROR: Could not write to file (%d), iteration %d\r\n", ret, i);
            f_close(&fil);
            break;
        }
        else
            written++;
    }
    if(written == 1280){
        ecg_saved = true;
        printf("Saved ECG\n");
    }
    
    // Close file
    fr = f_close(&fil);
}

void setup(){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("Entered setup!");
    
    gpio_init(UART_TX_PIN);
    gpio_set_dir(UART_TX_PIN, true);
    gpio_put(UART_TX_PIN, 0);

    gpio_init(UART_RX_PIN);
    gpio_set_dir(UART_RX_PIN,false);
    gpio_set_pulls(UART_RX_PIN,1,0);

    #if !debug

    bool uart = false, init = false;

    while(config){
        if(!gpio_get(UART_RX_PIN) && !uart){
            //uart
            gpio_put(UART_TX_PIN, 0);
            uart = true;
        }

        if(uart && !init){
            // Set the TX and RX pins by using the function select on the GPIO
            // Set datasheet for more information on function select
            gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
            gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

            // Set up our UART with the required speed.
            uart_init(UART_ID, BAUD_RATE);

            // habilito la recepcion por interrupciones
            uart0_hw->imsc = UART_UARTIMSC_RXIM_BITS;


            // configuro la interrupcion de la uart
            irq_set_exclusive_handler(UART0_IRQ, Recibe_car);
            irq_set_enabled(UART0_IRQ, true);
            init = true;
        }
    }
    #endif

    //*Save config to flash
    uint8_t conf = acel_habi << 5 | electro_habi << 4 | max_habi << 3 | wifi_hab << 2 | datalog << 1 | panico;
    uint8_t electro1 = (electro >> 8) & 0xFF, electro2 = electro & 0xFF;
    uint8_t spo_1 = (spo >> 8) & 0xFF, spo_2 = spo & 0xFF;

    uint8_t conf_data[FLASH_PAGE_SIZE] = {1};

    conf_data[0] = conf;
    conf_data[1] = electro1;
    conf_data[2] = electro2;
    conf_data[3] = spo_1;
    conf_data[4] = spo_2;

    int rc = flash_safe_execute(exec_flash_range_erase, (void *)ADDR_PERSISTENT_BASE_ADDR, 3000);

    if(rc != PICO_OK){
        printf("Error erasing flash: %d \n", rc);
        return;
    }


    uintptr_t params[] = { (uintptr_t)ADDR_PERSISTENT_BASE_ADDR, (uintptr_t)conf_data};

    rc = flash_safe_execute(exec_flash_range_program, params, 3000);

    if(rc != PICO_OK){
        printf("Error flashing configuration data: %d \n", rc);
    }

    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    printf("Config done");
    return;
}

void Recibe_car()
{

    if ((uart0_hw->ris & UART_UARTRIS_RXRIS_BITS) != 0) // Chequeo la interrupcion por RX
    {
        caracter_rec = uart0_hw->dr;
        uart0_hw->icr |= UART_UARTICR_RXIC_BITS; // Limpio la interrupcion de RX
    }


    if(caracter_rec =='!'){
        indice=0;
    }

    recibo[indice++]= caracter_rec;

    char trama[22]="!a-b-c-d-e-0000-0000";

    if(caracter_rec=='?'){
        // Acelerómetro
        if (recibo[1] == '1') {
            acel_habi = 1;
        }       
        else if (recibo[1] == '0') {
            acel_habi = 0;
        }

        // ADC
        if (recibo[3] == '1') {
            electro_habi=1;
        } 
        else if (recibo[3] == '0') {
            electro_habi=0;
        }

        // MAX30100
        if (recibo[5] == '1') {
            max_habi=1;
        } 
        else if (recibo[5] == '0') {
            max_habi=0;
        }

        // Wifi
        if (recibo[7] == '1') {
            wifi_hab = 1;
        } else if (recibo[7] == '0') {
            wifi_hab = 0;
        }

        // SD
        if (recibo[9] == '1') {
            datalog = 1;
        } else if (recibo[9] == '0') {
            datalog = 0;
        }

        electro= ((recibo[11]-48)*1000)+((recibo[12]-48)*100)+((recibo[13]-48)*10)+(recibo[14]-48);
        spo= ((recibo[16]-48)*1000)+((recibo[17]-48)*100)+((recibo[18]-48)*10)+(recibo[19]-48);
        
        // PANICO
        if (recibo[21] == '1') {
            panico = 1;
        } 
        else if (recibo[21] == '0') {
            panico = 0;
        }

        config = false;

    }
}

void exec_flash_range_erase(void* param){
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase((uint32_t)param, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    return;
}

void exec_flash_range_program(void* param){
    uint32_t addr = ((uintptr_t*)param)[0];
    uint8_t *data = (uint8_t *)((uintptr_t*)param)[1];

    flash_range_program(addr, data, FLASH_PAGE_SIZE);
}

static void sleep(uint32_t secs){
    sleep_run_from_lposc();
    struct timespec ts;
    aon_timer_get_time(&ts);
    ts.tv_sec += secs;

    sleep_goto_dormant_until(&ts, &sleep_callback);
}

int disp_init(){
    
    gpio_init(button);
    gpio_set_dir(button, 0);

    #if !debug

    uint8_t *data_ptr = (uint8_t*)ADDR_PERSISTENT_BASE_ADDR;

    //* Read configuration data from flash
    uint32_t ints = save_and_disable_interrupts();
    

    //memcpy(&loaded_config, (int *)ADDR_PERSISTENT_BASE_ADDR, 5);

    uint8_t loaded_conf[5] = {0};
    //!chcat

    // safe read: volatile to prevent compiler optimizing away
    const volatile uint8_t *p = (const volatile uint8_t *)persistent_addr;
    for (int i = 0; i < 5; ++i) {
        loaded_conf[i] = p[i];
    }
    //!chat
    // loaded_conf[0] = loaded_config >> 56;
    // loaded_conf[1] = (loaded_config >> 48) & 0xFF;
    // loaded_conf[2] = (loaded_config >> 40) & 0xFF;
    // loaded_conf[3] = (loaded_config >> 32) & 0xFF;
    // loaded_conf[4] = (loaded_config >> 24) & 0xFF;
    

    acel_habi = (loaded_conf[0] >> 5) & 1;
    electro_habi = (loaded_conf[0] >> 4) & 1;
    max_habi = (loaded_conf[0] >> 3) & 1;
    wifi_hab = (loaded_conf[0] >> 2) & 1;
    datalog = (loaded_conf[0] >> 1) & 1;
    panico = loaded_conf[0] & 1;
    electro = loaded_conf[1] << 8 | loaded_conf[2];
    spo = loaded_conf[3] << 8 | loaded_conf[4];
    //* Read configuration data from flash
    restore_interrupts(ints);
    #endif

    if(electro <= 10 || spo <= 10)
        mode_cont = true;

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    adc_hw->cs |= ADC_CS_START_ONCE_BITS;

    //*wifi trama mando
    //* !1-1-1-1-1-1-0000-0000-1? 

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 100*1000);
    
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    if(datalog){
        if (!sd_init_driver()) {
            printf("ERROR: Could not initialize SD card\r\n");
        }

        // Mount drive
        fr = f_mount(&fs, "0:", 1);
        if (fr != FR_OK) {
            printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        }
    }

    if(wifi_hab){
        cyw43_arch_enable_sta_mode();
        printf("Connecting to WiFi...\n");
        
        while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS,
                                            CYW43_AUTH_WPA2_AES_PSK, 30000)) {
            printf("Failed to connect.\n");
        }

        ip_addr_t server_addr;
        ip4addr_aton(SERVER_IP, &server_addr);

        printf("Connecting to server %s:%d...\n", SERVER_IP, SERVER_PORT);
        client_pcb = tcp_new();
        tcp_connect(client_pcb, &server_addr, SERVER_PORT, connected_callback);
    }
    else{
        cyw43_arch_deinit();
    }

    if(max_habi){
        MAX30100_init(&max, i2c_default, 0x57); //Sensor init

        MAX30100_cfg(&max, 0b11, 0, 0, 1, 1, false); //Sensor config, SPO2 mode, 200uS pulsewidth, 50Hz sample rate, 4.4mA for both leds, no high resolution
        MAX30100_Start_temp(&max); //Starts a temperature reading

        MAX30100_FIFO_CLR(&max); //!Clear the FIFO if this isn't done, the sensor doesn't work
        spo2_filter_init(&spo2f, 0x03, 0x03);
    }

    if(electro_habi){
        ads_init(&adsensor, i2c_default, addr);

        // Configure the ADS to use channel 0 only
        if (!ads_config_channel0(&adsensor)){
            printf("Failed to configure ADS channel 0\n");
        }

        ads_config_ch0_single_ended(&adsensor);

        // Optionally start the device data buffer/sequence if required by the sensor
        if (!ads_start(&adsensor)){
            printf("Warning: ads_start failed (device may not need explicit start)\n");
        }
    }

    if(acel_habi){
        start(false, 0x4C);
    }
        
    stdio_init_all();
}

// Initialise everything
void powman_init(uint64_t abs_time_ms) {
    // start powman and set the time
    powman_timer_start();
    powman_timer_set_ms(abs_time_ms);

    // Allow power down when debugger connected
    powman_set_debug_power_request_ignored(true);

    // Power states
    powman_power_state P1_7 = POWMAN_POWER_STATE_NONE;

    powman_power_state P0_3 = POWMAN_POWER_STATE_NONE;
    P0_3 = powman_power_state_with_domain_on(P0_3, POWMAN_POWER_DOMAIN_SWITCHED_CORE);
    P0_3 = powman_power_state_with_domain_on(P0_3, POWMAN_POWER_DOMAIN_XIP_CACHE);

    off_state = P1_7;
    on_state = P0_3;
}

// Initiate power off
static int powman_sleep(uint32_t secs) {

    hw_set_bits(&powman_hw->vreg_ctrl, POWMAN_PASSWORD_BITS | POWMAN_VREG_CTRL_UNLOCK_BITS);

    //powman_enable_gpio_wakeup(0, 2, false, true); //TODO Test if this works with the timer wakeup
    uint64_t ms = powman_timer_get_ms();
    powman_enable_alarm_wakeup_at_ms(ms+(secs*1000));

    // Get ready to power off
    stdio_flush();

    // Set power states
    bool valid_state = powman_configure_wakeup_state(off_state, on_state);
    if (!valid_state) {
        return PICO_ERROR_INVALID_STATE;
    }

    // Switch to required power state
    int rc = powman_set_power_state(off_state);
    if (rc != PICO_OK) {
        return rc;
    }

    // Power down
    while (true) __wfi();
}
