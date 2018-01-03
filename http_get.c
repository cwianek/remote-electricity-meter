/* http_get - Retrieves a web page over HTTP GET.
 *
 * See http_get_ssl for a TLS-enabled version.
 *
 * This sample code is in the public domain.,
 */
#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <unistd.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "ssid_config.h"

#define WEB_PORT 80
#define WEB_PATH "/"

#define Wh_PRICE 0.0006713 //cena za wato-godzinę
#define DEVICE_POWER 800

enum State {
	START, STOP, INIT
};
static enum State state = INIT;

static TickType_t currentStartTime = 0;
static TickType_t stopTime = 0;
static int billingInProgress = 0;
const int gpio = 1;


void enableDevice() {
	//to implement
}

void disableDevice() {
	//to implement
}

float costForEnergy(uint32_t ticks) {
	uint32_t passedTimeSec = (uint32_t)ticks / configTICK_RATE_HZ;
	float cost = (float)passedTimeSec / 60 / 60 * Wh_PRICE * DEVICE_POWER;
	return cost;
}

int handleCommand(char * command, char * response) {
    sprintf(response, "");
    if (strcmp("start", command) == 0) {
        void enableDevice();
        if (state == INIT || billingInProgress == 0) {
            billingInProgress = 1;
            currentStartTime = xTaskGetTickCount();
        }
        else if (state == STOP) {
            currentStartTime = xTaskGetTickCount() - stopTime + currentStartTime;
        }
        sprintf(response, "device started\n");
        state = START;
        gpio_write(gpio, 1);
    }
    else if (strcmp("stop", command) == 0) {
        if(state == START){
            void disableDevice();
            stopTime = xTaskGetTickCount();
        }
        sprintf(response, "device stopped\n");
        state = STOP;
        gpio_write(gpio, 0);
    }
    else if (strcmp("bill", command) == 0) {
        if (!currentStartTime) {
            sprintf(response, "device didn't started yet\n");
        }
        else {
            TimerHandle_t ticks;
            if(state == START){
                ticks = xTaskGetTickCount() - currentStartTime;
            }
            else {
                ticks = stopTime - currentStartTime;
            }
            float cost = costForEnergy(ticks);
            if (!billingInProgress) {
                cost = 0;
            }
            sprintf(response, "cost since start: %f zl\n",cost);
        }
    }
    else if (strcmp("pay", command) == 0) {
        if(state == STOP){
            billingInProgress = 0;
            sprintf(response, "paid\n");
        }
        else {
            sprintf(response, "you should stop device to pay the bills\n");
        }
    }
    else if (strcmp("exit", command) == 0) {
        sprintf(response, "bye\n");
        return -1;
    }
    else {
        sprintf(response, "invalid command \"%s\".\navailable commands are: start, stop, bill, pay, exit\n", command);
    }

    return 0;
}

void read_command(int s, char* buf) {
    int i;
    for (i = 0; i < 9; ++i) {
        read(s, buf + i, 1);
        if (buf[i] == '\n') {
            buf[i] = '\0';
            break;
        }
    }
    buf[10] = '\0';
}

void handler_task(int socket) {
    //mock - zamiast scanf-ow pewnie będzie jakiś odczyt z socketów 
	char command[10];
	char response[64];
        int ret;
        write(socket, "Hello!\n", 7);
	while (1) {
            write(socket, "> ", 2);
            read_command(socket, command);
            ret = handleCommand(command, response);
            write(socket, response, strlen(response));
            if (ret == -1) {
                break;
            }
	}
        close(socket);
        vTaskDelete(NULL);
}

void listen_task(void *pvParameters)
{
    int successes = 0, failures = 0;
    printf("server task starting...\n");

    struct sockaddr_in addr;
    size_t addrlen = sizeof(addr);
    memset(&addr, 0, addrlen);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(WEB_PORT);

    int socket = socket(AF_INET, SOCK_STREAM, 0);
    int err = errno;

    if (socket < 0) {
        printf("Error creating listen socket %d\n", err);
    }
    if (bind(socket, &addr, addrlen) != 0) {
        printf("Error binding\n");
    }
    if (listen(socket, 5) < 0) {
        printf("Error listening\n");
    }

    while(1) {
        printf("Listening...\n");
        int s = accept(socket, &addr, &addrlen);
        if (s < 0) {
            printf("Error accepting\n");
        } else {
            printf("Accepted incoming connection\n");
            xTaskCreate(&handler_task, "handler_task", 384, (void*) s, 2, NULL);
        }
    wait:
        vTaskDelay(1000 / portTICK_PERIOD_MS);

    }       
 close_socket:
    close(socket);
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    gpio_enable(gpio, GPIO_OUTPUT);
    
    xTaskCreate(&listen_task, "listen_task", 384, NULL, 2, NULL);
}

