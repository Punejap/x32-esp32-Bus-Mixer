

#include <stdlib.h>
#include <string.h>
#include <sys/param.h>
#include <inttypes.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"


#include "esp_system.h"
#include "esp_event.h"

#include "esp_log.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include <ssd1306.h>
#include <i2cdev.h>

#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

//mines
#include "my_pcnt.h"

//IP stuff
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#define PORT CONFIG_EXAMPLE_PORT
//rotary encoder stuff
#define PCNT_HIGH_LIMIT 4
#define PCNT_LOW_LIMIT  -4
//btn stuff
#define ROT_GPIO_A1 35  //S3 vvv 1
#define ROT_GPIO_A2 34  //2
#define ROT_GPIO_B1 33  //4
#define ROT_GPIO_B2 32  //5
#define ROT_GPIO_C1 15  //6
#define ROT_GPIO_C2 19  //7
#define ROT_GPIO_D1 18  //8
#define ROT_GPIO_D2 4   //9
//#define GPIO_OUTPUT_IO_0    18
//#define GPIO_OUTPUT_IO_1    19
//#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))

#define GPIO_INPUT_IO_0     12   //pg down btn  S3 12
#define GPIO_INPUT_IO_1     14  //pg up btn         13
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0
//x32 specifics
#define NUM_OF_CHANNELS 16

//i2c stuff
#define SDA_GPIO 21 //17 FOR S3
#define SCL_GPIO 22 //18 FOR S3

//led stuff
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      23  //S3 16
#define EXAMPLE_LED_NUMBERS         64
#define EXAMPLE_CHASE_SPEED_MS      1000



static const char *TAG = "jawn";

struct sockaddr_in x32_address; 

uint8_t page_num;
int sock;

typedef struct channel_array{
    char ch_names[16][12];
    int fader_values[16];
    //uint8_t page_num;
} channel_array;

typedef struct IO_data_packet{
    int32_t channel_num;
    int32_t fader_val;
} IO_data_packet;


int encoder_arg[] = {1,2,3,4};

static QueueHandle_t xOutgoing_Data_Queue;
static QueueHandle_t xRotary_Data_Queue;

static TaskHandle_t xFader_Control_Handle;                                      //handles led ring. init by reading fader vals and outputting. gets notified on page change.
static TaskHandle_t xPoll_Fader_Data_Handle;                                                 //poll for ch val updates from x32 
static TaskHandle_t xReceive_Data_Handle;
static TaskHandle_t xSend_Data_Handle;                                                //sends messages to x32 from xOutput_data_queue
static TaskHandle_t xOLED_Control_Handle;
static TaskHandle_t xLED_Ring_Handle;

static SemaphoreHandle_t xMutex = NULL;

void vRemote(void *pvParameters);
void vPoll_Data(void *pvParameters);
void vReceive_Data(void *pvParameters);
void vSend_Data(void *pvParameters);
void vFader_Control(void *pvParameters);
static bool pcnt_isr_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx);
void vOLED_Control(void *pvParameters);
void vLED_Ring_Control(void *pv);

void inline Boof_Ring(uint8_t *sliced_pixels, int i);


inline int32_t Reverse32(int32_t value) 
{
    return (((value & 0x000000FF) << 24) |
            ((value & 0x0000FF00) <<  8) |
            ((value & 0x00FF0000) >>  8) |
            ((value & 0xFF000000) >> 24));
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    uint32_t gpio_num = (uint32_t) arg;
    //do math change pagenum
    gpio_intr_disable(12);
    gpio_intr_disable(14);
    if (gpio_num==12 && page_num<3 )
    {
        page_num+=1;
    }
    else if(gpio_num ==14 && page_num > 0)
    {
        page_num-=1;
    }
    vTaskNotifyGiveFromISR(xOLED_Control_Handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

channel_array *bus0 = {0};

pcnt_event_callbacks_t cbs = {
    .on_reach = pcnt_isr_handler,
};


void app_main(void)
{
    x32_address.sin_family=AF_INET; 
    x32_address.sin_port=htons(10023);
    x32_address.sin_addr.s_addr=(inet_addr("192.168.1.165"));
    bus0 = (channel_array*)malloc(sizeof(channel_array));
    for(size_t i=0; i<16; i++)
    {
        for(size_t j=0; j<12; j++)
        {
            bus0->ch_names[i][j] = ' ';
        }
    }

    page_num = 0;
    
    gpio_config_t io_conf = {};
    // //disable interrupt
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // //set as output mode
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // //bit mask of the pins that you want to set,e.g.GPIO18/19
    // io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    // //disable pull-down mode
    // io_conf.pull_down_en = 0;
    // //disable pull-up mode
    // io_conf.pull_up_en = 0;
    // //configure GPIO with the given settings
    // gpio_config(&io_conf);

    gpio_set_level(GPIO_INPUT_IO_0, 0);
    gpio_set_level(GPIO_INPUT_IO_1, 0);
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //enable pulldowns
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_POSEDGE);

    

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    // uint16_t pg_up = 1;
    // uint16_t pg_dwn = 0;
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

    //////////////////////////

    pcnt_unit_handle_t encoder1 = create_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, ROT_GPIO_A1, ROT_GPIO_A2 );
    pcnt_unit_handle_t encoder2 = create_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, ROT_GPIO_B1, ROT_GPIO_B2 );
    pcnt_unit_handle_t encoder3 = create_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, ROT_GPIO_C1, ROT_GPIO_C2 );
    pcnt_unit_handle_t encoder4 = create_encoder(PCNT_HIGH_LIMIT, PCNT_LOW_LIMIT, ROT_GPIO_D1, ROT_GPIO_D2 );


     int watch_points[] = {PCNT_LOW_LIMIT, PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) 
    {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder1, watch_points[i]));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder2, watch_points[i]));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder3, watch_points[i]));
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder4, watch_points[i]));
    }


    
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(encoder1, &cbs, (void*) encoder_arg[0]));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(encoder2, &cbs, (void*) encoder_arg[1]));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(encoder3, &cbs, (void*) encoder_arg[2]));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(encoder4, &cbs, (void*) encoder_arg[3]));
    start_encoder(encoder1);
    start_encoder(encoder2);
    start_encoder(encoder3);
    start_encoder(encoder4);
    
    ////////////////////////////
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    ESP_ERROR_CHECK(i2cdev_init());

    xOutgoing_Data_Queue = xQueueCreate(32, sizeof(IO_data_packet));
    xRotary_Data_Queue = xQueueCreate(32, sizeof(IO_data_packet));
    xMutex = xSemaphoreCreateMutex();

    xTaskCreate(vRemote, NULL, 4096, NULL,6,NULL);
    xTaskCreate(vReceive_Data, NULL, 4096, (void *) bus0, 5, &xReceive_Data_Handle);
    xTaskCreate(vOLED_Control, NULL, 4096, (void *) bus0, 5, &xOLED_Control_Handle);
    xTaskCreate(vSend_Data, NULL, 4096, NULL, 4, &xSend_Data_Handle);
    xTaskCreate(vFader_Control, NULL, 4096, (void *) bus0, 4, &xFader_Control_Handle);
    xTaskCreate(vLED_Ring_Control, NULL, 4096, (void *) bus0, 4, &xLED_Ring_Handle);
    xTaskCreate(vPoll_Data, NULL, 4096, NULL,4, &xPoll_Fader_Data_Handle);
}

void vRemote(void *pvParameters)
{   
    char rem_buf[12]="/xremote\0\0\0\0";
        
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
    }
    
    while(1){
        sendto(sock, rem_buf, sizeof(rem_buf), 0, (struct sockaddr *)&x32_address, sizeof(x32_address));
        vTaskPrioritySet(NULL, 3);
        vTaskDelay(pdMS_TO_TICKS(8000));
    }
}

//updates fader values from x32
void vPoll_Data(void *parameter)
{
    int index = 0;
    char tx_buf[20] = "/ch/00/mix/03/level\0";
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    
    while(1)
    {  
        //loop thru each channel
        for(int i = 0; i< 16; i++)
        {
            //compensate for single array element manipulation
            index = i + 1;
            if(index>=10)
            {
                tx_buf[4] = '1';
                index -= 10;
            }else tx_buf[4] = '0';
            tx_buf[5] = '0' + index;

            //ESP_LOGI(TAG, "POLL %s", tx_buf);
            sendto(sock, tx_buf, sizeof(tx_buf), 0, (struct sockaddr *)&x32_address, sizeof(x32_address));   
            ulTaskNotifyTake(pdFALSE,pdMS_TO_TICKS(100));
            
        }
        vTaskPrioritySet(NULL,3);
        vTaskPrioritySet(xReceive_Data_Handle, 3);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void vReceive_Data(void *parameter)
{
    struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
    socklen_t socklen = sizeof(source_addr);
    channel_array *bus = (channel_array *) parameter;
    char rx_buf[32];

    int curr_poll_val_i;
    int curr_poll_ch;
    float f_val;
        
    while(1)
    {
        for(size_t i=0; i<32; i++){
            rx_buf[i] = ' ';
        }
        curr_poll_val_i = 0;
        f_val=0;
        curr_poll_ch = 0;
        //try to get channel data until timer exceeds
       // while(recvfrom(sock, rx_buf, sizeof(rx_buf), 0, (struct sockaddr *)&source_addr,&socklen) ==-1 )

        recvfrom(sock, rx_buf, sizeof(rx_buf), 0, (struct sockaddr *)&source_addr,&socklen);

            //get ch num
            curr_poll_ch = rx_buf[5] - '0';
            if(rx_buf[4]=='1')
            {
                curr_poll_ch += 10;
            }
            if( strncmp( (rx_buf+14), "level", 5) == 0 )
            {
                //get fader val
                
                memcpy(&curr_poll_val_i, &rx_buf[24], 4);
                curr_poll_val_i = Reverse32(curr_poll_val_i);
                memcpy(&f_val, &curr_poll_val_i, 4);
                curr_poll_val_i = f_val * 1024;
                
                //on the queue for fader task to receive
                //maybe do this: if(curr_poll_ch == i)
                if(xSemaphoreTake(xMutex, portMAX_DELAY) == 1)
                {
                    bus->fader_values[ (curr_poll_ch - 1) ]= curr_poll_val_i;
                    xSemaphoreGive(xMutex);
                }
                xTaskNotifyGive(xPoll_Fader_Data_Handle);
                xTaskNotifyGive(xLED_Ring_Handle);
            }else if( strncmp( (rx_buf+14), "name", 4) == 0)
            {
                //memcpy(&curr_poll_val_i, &rx_buf[19], 5);
                //curr_poll_val_i = Reverse32(curr_poll_val_i);
                //memcpy(bus->ch_names[ (curr_poll_ch - 1)], &curr_poll_val_i, 5);
                if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
                {
                    strcpy(bus->ch_names[ (curr_poll_ch - 1)], &rx_buf[24]);    
                    xSemaphoreGive(xMutex);
                }
                xTaskNotifyGive(xOLED_Control_Handle);   
            }
    }
}


void vSend_Data(void *pvParameters)
{
    unsigned char tx_buf[28] = "/ch/00/mix/03/level\0,i\0\0000000";

    IO_data_packet Out_Data = {0};
    int32_t fader_val_out;

    while(1)
    {
        if( xQueueReceive(xOutgoing_Data_Queue, &Out_Data, portMAX_DELAY) == pdPASS)
        {
            if(Out_Data.channel_num >= 10)
            { 
                tx_buf[4] = '1';
                tx_buf[5] = (Out_Data.channel_num - 10) + '0';
            }else tx_buf[5] = Out_Data.channel_num + '0';

            fader_val_out = Reverse32(Out_Data.fader_val);

            memcpy(&tx_buf[24], &fader_val_out, 4);
            //ESP_LOGI(TAG,"trying to send %ld, %s", fader_val_out, tx_buf);
            //ESP_LOGI(TAG,"sending fader value reverso %d", tx_buf[24]);
            sendto(sock, tx_buf, sizeof(tx_buf), 0, (struct sockaddr *)&x32_address, sizeof(x32_address));

        }
    }
}

static bool IRAM_ATTR pcnt_isr_handler(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
   // ESP_LOGI(TAG,"interupt");
    int32_t chnum = (int32_t) user_ctx;
   IO_data_packet RE_Data = {
    .channel_num = chnum,
    .fader_val = edata->watch_point_value
   };
    
    xQueueSendFromISR(xRotary_Data_Queue, &RE_Data, &xHigherPriorityTaskWoken);
    //portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
   
    return (xHigherPriorityTaskWoken == pdFALSE);
}

void vFader_Control(void *parameter)
{
    channel_array *bus = (channel_array *) parameter;

    int32_t channel_num;
    float poop;
    IO_data_packet update = {0};

    while(1)
    {
        if(xQueueReceive(xRotary_Data_Queue, &update, portMAX_DELAY) == pdPASS)
        {
            
            channel_num = update.channel_num + 4 * page_num;
            if(xSemaphoreTake(xMutex,portMAX_DELAY))
            {
                
                bus->fader_values[channel_num-1] += update.fader_val * 8;
                poop =  (bus->fader_values[channel_num-1])/6.4;

                if(bus->fader_values[channel_num-1] > 1024) bus->fader_values[channel_num-1] = 1024;
                if(bus->fader_values[channel_num-1] < 0 ) bus->fader_values[channel_num-1] = 0;
                if(poop > 160) poop = 160;
                if(poop < 0) poop = 0;
                //ESP_LOGI(TAG,"prior - ch num %lu, fa val %d pagenum %d",update.channel_num, bus->fader_values[channel_num-1], page_num);
                xTaskNotifyGive(xLED_Ring_Handle);

                update.channel_num = channel_num;
                update.fader_val = poop;
                xSemaphoreGive(xMutex);
            }
            
            xQueueSend(xOutgoing_Data_Queue, &update, pdMS_TO_TICKS(100));
        }
    }
}

void vOLED_Control(void *parameter)
{
    //ESP_LOGI(TAG,"trying to send oled");

    
    channel_array *bus = (channel_array *) parameter;

    //multiplexer
    i2c_dev_t mplexer = {0};
    mplexer.cfg.sda_io_num = 21;
    mplexer.cfg.scl_io_num = 22;
    mplexer.cfg.master.clk_speed = 400000; //400khz
    mplexer.addr = 0x70;
    uint8_t mplex_cmds[] = {0b01110000, 0b00000001};

    //OLED screen
    SSD1306_t OLED;
    OLED._address = 0x3c;
    OLED._flip = false;
    //ESP_LOGI(TAG,"OLED b4 FOR LOOP");
    char tx_buf[] = {"/ch/00/config/name"};


    for(int i = 1; i <=16; i++)
    {
        
        if(i<10){

         tx_buf[5] = '0' + i;
        }else
        {
            tx_buf[4] = '1';
            tx_buf[5] = '0' + (i-10);
        }

        
        sendto(sock, tx_buf, sizeof(tx_buf), 0, (struct sockaddr *)&x32_address, sizeof(x32_address));

        //vTaskDelay(pdMS_TO_TICKS(500));
        ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1000));//pdMS_TO_TICKS(1000));
    }

    vTaskPrioritySet(NULL, 4);
    
    xTaskNotifyGive(xLED_Ring_Handle);
    xTaskNotifyGive(xPoll_Fader_Data_Handle);

    while(true)
    {
        mplex_cmds[1] = 0b00000001;
        for(int i=0; i<4; i++)
        {     
            i2c_dev_write_reg(&mplexer, mplexer.addr, &mplex_cmds, sizeof(mplex_cmds));
            ssd1306_init(&OLED, 128, 32);
            if(xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE)
            {
                ssd1306_clear_screen(&OLED, false);
                ssd1306_display_text_x3(&OLED, 0, bus->ch_names[i + 4*page_num], sizeof(bus->ch_names[i + 4*page_num]), false);
                
                xSemaphoreGive(xMutex);
            }

            mplex_cmds[1] = mplex_cmds[1] << 1;
        }
        //block until page btn pressed
        gpio_intr_enable(12);
        gpio_intr_enable(14);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

}



void vLED_Ring_Control(void *pv)
{
    ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
    uint32_t vol;   //0<i<13 for each channel
    uint32_t divider = 79;
    uint8_t led_pixels[EXAMPLE_LED_NUMBERS * 3];

    //install led hardware
    channel_array *bus = (channel_array *) pv;
    rmt_channel_handle_t led_ch = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
        .gpio_num = RMT_LED_STRIP_GPIO_NUM,
        .mem_block_symbols = 64, // increase the block size can make the LED less flickering
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_ch));
    
    rmt_encoder_handle_t led_encoder = NULL;
    led_strip_encoder_config_t encoder_config = {
        .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

    //ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(led_ch));

    //ESP_LOGI(TAG, "Start LED rainbow chase");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0, // no transfer loop
    };

    while(true)
    {
        ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
        for(size_t i=0; i<4; i++)
        {   
            if(xSemaphoreTake(xMutex, portMAX_DELAY)==pdTRUE)
            {
                vol = bus->fader_values[i+4*page_num] / divider;
                Boof_Ring(&led_pixels[i*16*3], vol);
                xSemaphoreGive(xMutex);
            }
        }

        ESP_ERROR_CHECK(rmt_transmit(led_ch, led_encoder, led_pixels, sizeof(led_pixels), &tx_config));
        ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_ch, portMAX_DELAY));
        
        
        //vTaskDelay(pdMS_TO_TICKS(100));
        
        vTaskPrioritySet(NULL, 3);
    }
}

//0<vol<13
void Boof_Ring(uint8_t sliced_pixels[],int vol)
{

    uint8_t colors[3][3] = { {6,0,0}, {4,12,0} ,{0,10,0} };
    uint16_t green = 0;
    uint16_t orange = 1;
    uint16_t red = 2;
    uint8_t off[] = {0,0,0};

    for(size_t i=0; i<16; i++)
    {
        if( (i==0) && (vol<=2) )
        {
            memcpy(&sliced_pixels[i*3], &off, 3 * sizeof(uint8_t));
        }
        if( (i<=6) && (i<=vol) )
        {
            memcpy(&sliced_pixels[i*3], &colors[green][0], 3 * sizeof(uint8_t));
        }

        else if( (i<=10) && (i<=vol) )
        {
            memcpy(&sliced_pixels[i*3], &colors[orange][0], 3 * sizeof(uint8_t));
        }
        else if( (i<=13) && (i<=vol) )
        {
           memcpy(&sliced_pixels[i*3], &colors[red][0], 3 * sizeof(uint8_t));
        }
        else if(i>vol)
        {
            memcpy(&sliced_pixels[i*3], &off, 3 * sizeof(uint8_t));
        }
    }
}

