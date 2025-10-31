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
#include "max30100_spo2.h"


//TODO check red comments and other TODO's

//Flash
extern uint32_t ADDR_PERSISTENT[];
#define ADDR_PERSISTENT_BASE_ADDR (ADDR_PERSISTENT)

//wifi
#define WIFI_SSID "Labyrinth"
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
//Intervals
uint16_t electro=6000,spo=12000;
uint16_t samples = 0;

//ECG
float ecg[1280] = {0};
bool ecg_done = false;

//Battery
float bat_val = 0;
uint64_t last_bat_rd = 0;
const uint64_t bat_rd_t 5 * 60e6;

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

    if(rec_data[0] == '1')
        blink = 1;
    else if(rec_data[0] == '0')
        blink = 0;
    
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

void send_to_server(const char *msg) {
    if (client_pcb) {
        tcp_write(client_pcb, msg, strlen(msg), TCP_WRITE_FLAG_COPY);
        tcp_output(client_pcb);
        printf("[You]: %s\n", msg);
    }
}
//Funciones wifi //TODO change this to use lib functions


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
uint8_t electro_habi = 1, acel_habi = 0, max_habi = 1, cayo = 0, panico = 0, datalog = 1, wifi_hab = 0;
uint16_t ch0;
uint64_t t = 0, last_t = 0, samp_t = 0;

MAX30100_t max; //Sensor struct
float spo2 = 0;
bool max_done = false, max_saved = false;
spo2_filter_t spo2f;
uint16_t ir[500] = {0}, red[500] = {0};
uint16_t max_samp = 0;
uint64_t last_max_t = 0, last_spo2t = 0;
// callback to update LED currents
void set_led_current(uint8_t ir, uint8_t red) {
    MAX30100_cfg(&sensor, 0x03, 0x03, 0x01, ir, red, true);
}

void Recibe_car(); 
void setup();

FRESULT fr;
FATFS fs;
FIL fil;
int ret;
char buf[100];
char filename_ecg[] = "ecg.csv";
char filename_spo[] = "spo2.csv";

int main()
{
    stdio_init_all();

    gpio_init(button);
    gpio_set_dir(button, 0);

    if(gpio_get(2)){
        config = true;
        setup();
    }

    //* Read configuration data from flash
    uint8_t loaded_conf[5] = {0};

    memcpy(loaded_conf, ADDR_PERSISTENT_BASE_ADDR, 5);

    uint8_t conf = acel_habi << 5 | electro_habi << 4 | max_habi << 3 | wifi_hab << 2 | datalog << 1 | panico;
    acel_habi = (loaded_conf[0] >> 5) & 1;
    electro_habi = (loaded_conf[0] >> 4) & 1;
    max_habi = (loaded_conf[0] >> 3) & 1;
    wifi_hab = (loaded_conf[0] >> 2) & 1;
    datalog = (loaded_conf[0] >> 1) & 1;
    panico = loaded_conf[0] & 1;
    electro = loaded_conf[1] << 8 | loaded_conf[2];
    spo = loaded_conf[3] << 8 | loaded_conf[4];
    //* Read configuration data from flash

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
    adc_hw->cs |= ADC_CS_START_ONCE_BITS;


    
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed");
        return -1;
    }

    //*wifi trama mando
    //* !1-1-1-1-1-1-0000-0000-1? 

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
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


    while (true) {

        t = timer0_hw->timehr<<32 | timer0_hw->timelr;

        if(t > (last_bat_rd + bat_rd_t)){ 
            adc_init();
            adc_gpio_init(26);
            adc_select_input(0);
            adc_hw->cs |= ADC_CS_START_ONCE_BITS;

            last_bat_rd = t;
        }

        if(adc_hw->cs & ADC_CS_READY_BIT){
            bat_val = valor = ((adc_hw->result * 2)*3.3)/4096;
        }

        if(bat_val <= bat_min_lvl){
            //TODO Shutdown
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

            sleep_ms(200);
        }

        //* take ecg
        if(electro_habi && (t/1e6)>((last_t/1e6)+electro)){
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

        if(max_habi){ //TODO make this take enough measurements to be able to calculate spo2
            if(MAX30100_read(&max)){

                if(MAX30100_read_temp(&max)){
                    MAX30100_Start_temp(&max);
                }
                MAX30100_FIFO_CLR(&max);

                if(t > (last_max_t + 20e3)){
                    red[max_samp++] = sensor.ir;
                    ir[max_samp++] = sensor.red;
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
                max_saved = true;
                last_spo2t = t;
            }
        }

        if(wifi_hab){
            cyw43_arch_poll();
            if ((client_pcb && max_done && ecg_done)||(client_pcb && panic_pressed)) { //! this should be data ready flag or smth
                
                //* wifi trama recibo
                //* !ooooooo-bbbbbbb-p-c-eeeeeeee(x1280)?
                //? oxigeno-bateria-panico-caida-electro
                

                send_to_server(data); //TODO make datagram
                sent = 1;
            }
        }

        if(sent){
            //TODO sleep until next interval
        }
    }
}

void get_adc_sample(){
    if(t > samp_t+(1e6/128)){
        ads_read_channel0(&adsensor, &ch0);
        samp_t = t;
        samples++;
        ecg[samples] = ch0;
    }
    if(samples == 1280){
        last_t = timer0_hw->timehr<<32 | timer0_hw->timelr;
        ecg_done = true;
        if(datalog) save_ecg();
    }
}

void save_spo(){
    // Open file for writing ()
    fr = f_open(&fil, filename_spo, 'a');
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
    }

    ret = f_printf(&fil, "%d", (uint16_t)spo2);
    if (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
    }

    // Close file
    fr = f_close(&fil);
}

void save_ecg(){
    // Open file for writing ()
    fr = f_open(&fil, filename_ecg, 'a');
    if (fr != FR_OK) {
        printf("ERROR: Could not open file (%d)\r\n", fr);
    }

    ret = f_printf(&fil, "%f", ecg);
    if (ret < 0) {
        printf("ERROR: Could not write to file (%d)\r\n", ret);
        f_close(&fil);
    }

    // Close file
    fr = f_close(&fil);
}

void setup(){
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    
    gpio_init(UART_TX_PIN);
    gpio_set_dir(UART_TX_PIN, true);
    gpio_put(UART_TX_PIN, 1);

    gpio_init(UART_RX_PIN);
    gpio_set_dir(UART_RX_PIN,false);
    gpio_set_pulls(UART_RX_PIN,1,0);

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
    flash_range_erase((uint32_t)param, FLASH_SECTOR_SIZE);
    return;
}

void exec_flash_range_program(void* param){
    uint32_t addr = ((uintptr_t*)param)[0];
    uint8_t *data = (uint8_t *)((uintptr_t*)param)[1];

    flash_range_program(addr, data, FLASH_PAGE_SIZE);
}