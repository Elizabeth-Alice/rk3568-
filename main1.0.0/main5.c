#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include "cJSON.h"
#include <pthread.h>
//#include <mosquitto.h> // paho-mqtt-c头文件


#define MQTT_BROKER "127.0.0.1"  // Mosquitto只需IP，无需tcp://前缀
#define MQTT_BROKER_PORT 1883    // Mosquitto默认端口
// #define MQTT_CLIENT_ID "rk3568_zigbee_gateway"
// #define MQTT_TOPIC_UP_SENSOR "sensor/up/data" // 传感器数据上报主题
// #define MQTT_TOPIC_UP_DEVICE "device/up/state" // 设备状态上报主题

int zigbee_fd = -1;
#define ZIGBEE_DEV "/dev/ttyS4"
#define RECV_TIMEOUT 500  // 新增：超时时间 500ms，可根据需求调整

// struct mosquitto *mqtt_client = NULL; // Mosquitto客户端句柄
// // MQTT初始化函数
// int mqtt_init(void) {
//     // 1. 初始化Mosquitto库
//     if (mosquitto_lib_init() != MOSQ_ERR_SUCCESS) {
//         printf("Mosquitto库初始化失败\n");
//         return -1;
//     }

//     // 2. 创建客户端实例（参数：客户端ID、是否清理会话、用户数据）
//     mqtt_client = mosquitto_new(MQTT_CLIENT_ID, true, NULL);
//     if (mqtt_client == NULL) {
//         printf("Mosquitto客户端创建失败\n");
//         mosquitto_lib_cleanup();
//         return -2;
//     }
//     // 3. 连接MQTT Broker（参数：客户端、Broker地址、端口、心跳超时）
//     int rc = mosquitto_connect(mqtt_client, MQTT_BROKER, MQTT_BROKER_PORT, 60);
//     if (rc != MOSQ_ERR_SUCCESS) {
//         printf("MQTT连接失败: %d\n", rc);
//         mosquitto_destroy(mqtt_client);
//         mosquitto_lib_cleanup();
//         return -3;
//     }
//     printf("MQTT连接成功（Mosquitto客户端）\n");
//     return 0;
// }
// // MQTT发布消息函数（适配Mosquitto API）
// int mqtt_publish_msg(const char* topic, const char* payload) {
//     if (mqtt_client == NULL || topic == NULL || payload == NULL) {
//         printf("MQTT发布参数无效\n");
//         return -1;
//     }
//     // 参数：客户端、消息ID（NULL自动分配）、主题、有效载荷长度、有效载荷、QoS、是否保留
//     int rc = mosquitto_publish(
//         mqtt_client, 
//         NULL, 
//         topic, 
//         strlen(payload), 
//         payload, 
//         1,  // QoS=1（与原Paho保持一致）
//         0   // 不保留消息
//     );

//     if (rc != MOSQ_ERR_SUCCESS) {
//         printf("MQTT发布失败: %d\n", rc);
//         return -2;
//     }
//     return 0;
// }

pthread_mutex_t zigbee_mutex = PTHREAD_MUTEX_INITIALIZER;
// 设备状态结构体（存储所有外设状态）
typedef struct {
    int led_state;        // LED状态：0-关，1-开
    int led_brightness;   // LED亮度：0-100
    int fan_state;           // 风扇状态：0-关，1-开
    int fan_speed;           // 风扇风速：0-100
    int fan_dir;             // 风扇方向：0-正转，1-反转
    float temp;              // 温度
    float humi;              // 湿度
} DeviceState;
DeviceState dev_state = {0};

/* 设置串口参数的函数 */
int set_uart(int fd, int speed, int bits, char check, int stop) {
    struct termios newtio, oldtio;
    // 步骤一：保存原来的串口配置
    if(tcgetattr(fd, &oldtio) != 0) {
        printf("tcgetattr oldtio error\n");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));
    // 步骤二：设置控制模式标志
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_lflag &= ~(ECHO | ECHOE | ISIG); // 禁用回显、禁止输入字符处理、禁止信号
    newtio.c_oflag &= ~OPOST; // 禁用输出处理（避免驱动对发送数据做额外处理）
    // 步骤三：设置数据位
    switch(bits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }
    // 步骤四：设置奇偶校验位
    switch(check) {
        case 'O': // 奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E': // 偶校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'N': // 无校验
            newtio.c_cflag &= ~PARENB;
            break;
    }
    // 步骤五：设置波特率
    switch(speed) {
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
    }
    // 步骤六：设置停止位
    switch(stop) {
        case 1:
            newtio.c_cflag &= ~CSTOPB; // 1位停止位
            break;
        case 2:
            newtio.c_cflag |= CSTOPB; // 2位停止位
            break;
    }
    // 步骤七：刷新输入队列
    tcflush(fd, TCIFLUSH);
    // 步骤八：设置配置立刻生效
    if (tcsetattr(fd, TCSANOW, &newtio) != 0) {
        printf("tcsetattr newtio error\n");
        return -2;
    }
    return 0;
}

//串口初始化函数
int zigbee_uart_init(void) {
    zigbee_fd = open(ZIGBEE_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (zigbee_fd < 0) {
        perror("ZigBee串口打开失败");
        return -1;
    }
    if(set_uart(zigbee_fd, 115200, 8, 'N', 1) != 0) {
        printf("uart set param error!\n");
        close(zigbee_fd);
        return -2;
    }
     printf("start recv full JSON...\n");
     return 0;
}


//json解析函数
void parse_zigbee_json(char *json_buf){
    if(json_buf == NULL || strlen(json_buf) == 0) {
        return;
    }
    cJSON *root = cJSON_Parse(json_buf);
    if (!root) {
        printf("JSON解析失败: %s\n", cJSON_GetErrorPtr());
        return;
    }

    cJSON *source = cJSON_GetObjectItem(root, "source");
    cJSON *driver = cJSON_GetObjectItem(root, "driver");
    cJSON *id     = cJSON_GetObjectItem(root, "id");
    cJSON *data   = cJSON_GetObjectItem(root, "data");
    // 必须有 来源+传感器类型+传感器ID 才继续处理
    if (!source || !driver || !id || !data || !source->valuestring || !driver->valuestring) {
        //LOG_WARN("JSON字段缺失: %s", json_buf);
        cJSON_Delete(root);
        return;
    }
    //pthread_mutex_lock(&zigbee_mutex);
    // DHT11温湿度解析
    if (strcmp(driver->valuestring, "dht11") == 0) {
        cJSON *humi_int = cJSON_GetObjectItem(data, "humi_int");
        cJSON *humi_deci = cJSON_GetObjectItem(data, "humi_deci");
        cJSON *temp_int = cJSON_GetObjectItem(data, "temp_int");
        cJSON *temp_deci = cJSON_GetObjectItem(data, "temp_deci");
        if (humi_int && humi_deci && temp_int && temp_deci) {
            dev_state.temp = temp_int->valueint + temp_deci->valueint / 10.0;
            dev_state.humi = humi_int->valueint + humi_deci->valueint / 10.0;
            printf("[DHT11-ID:%d] 温度: %.1f°C, 湿度: %.1f%%", id->valueint, dev_state.temp, dev_state.humi);
        }
        // printf("[%s-ID:%d] 温湿度数据已转发MQTT → %s\n", driver->valuestring, id->valueint, MQTT_TOPIC_UP_SENSOR);
        //mqtt_publish_msg(MQTT_TOPIC_UP_SENSOR, json_buf);
    }else if (strcmp(driver->valuestring, "led_pwm") == 0) {
        cJSON *led_state = cJSON_GetObjectItem(data, "led_state");
        cJSON *brightness = cJSON_GetObjectItem(data, "led_brightness");
        if (led_state && brightness) {
            dev_state.led_state = led_state->valueint;
            dev_state.led_brightness = brightness->valueint;
            printf("[LED-ID:%d] 状态: %d, 亮度: %d%%", id->valueint, led_state->valueint, brightness->valueint);
        }
        // printf("[%s-ID:%d] 温度数据已转发MQTT → %s\n", driver->valuestring, id->valueint, MQTT_TOPIC_UP_SENSOR);
        //mqtt_publish_msg(MQTT_TOPIC_UP_DEVICE, json_buf);
    }
    //pthread_mutex_unlock(&zigbee_mutex);
    cJSON_Delete(root);
}

void* uart_recv_thread(void* arg){
    // 缓存分段数据（要足够大，避免JSON包超过长度）
    char recv_buf[1024] = {0};
    // 单次read的临时缓冲区
    char temp_buf[256] = {0};
    int recv_len = 0; // 缓存区已存数据长度
    int count;
    //time_t last_recv_time = time(NULL);//记录最后一次收数据的时间
    //if (zigbee_uart_init() < 0) pthread_exit((void*)-1);
    while(1) {
        pthread_mutex_lock(&zigbee_mutex);
        // 读取分段数据到临时缓冲区
        memset(temp_buf, 0, sizeof(temp_buf));
        count = read(zigbee_fd, temp_buf, sizeof(temp_buf)-1);
        if(count > 0) {
            temp_buf[count] = '\0';
            // 修复1：先拼接数据，再判断溢出（避免误删有效数据）
            int remaining = sizeof(recv_buf) - recv_len - 1;
            if (remaining > 0) {
                strncat(recv_buf, temp_buf, remaining);
                recv_len += count;
            }
            // 缓冲区溢出保护：若即将填满，清空重收
            if(recv_len >= sizeof(recv_buf)-1) {
                printf("缓冲区已满且无结束符，清空重收\n");
                memset(recv_buf, 0, sizeof(recv_buf));
                recv_len = 0;
                pthread_mutex_unlock(&zigbee_mutex);
                continue;
            }
            // 更新最后一次收数据的时间
            //last_recv_time = time(NULL);
            // 检测缓存区中是否包含结束标志（\n）
            char *end_flag = strstr(recv_buf, "\r\n");
            if(end_flag != NULL) {
                *end_flag = '\0';
                printf("完整JSON数据：%s\n", recv_buf);
                parse_zigbee_json(recv_buf);
                // 清空缓存，准备下一包
                memset(recv_buf, 0, sizeof(recv_buf));
                recv_len = 0;
            }
        }
        // 超时清空机制，解决异常断包卡死问题
        // time_t now = time(NULL);
        // if((now - last_recv_time) * 1000 > RECV_TIMEOUT && recv_len > 0) {
        //     printf("数据超时未收到结束符，清空缓冲区，当前缓存：%s\n", recv_buf);
        //     memset(recv_buf, 0, sizeof(recv_buf));
        //     recv_len = 0;
        //     last_recv_time = now;
        // }
        pthread_mutex_unlock(&zigbee_mutex);
        usleep(10000); // 短延时，降低CPU占用
    }
    //close(zigbee_fd);
    return 0;
}


int main(int argc, char *argv[]) {
    // 初始化Zigbee串口
    if (zigbee_uart_init() < 0) {
        //LOG_FATAL("Zigbee串口初始化失败");
        return -1;
    }
    // // 初始化MQTT
    // if (mqtt_init() < 0) {
    //     //LOG_WARN("MQTT初始化失败，继续运行");
    // }
    // 创建线程
    pthread_t uart_thread, web_thread, mqtt_thread;
    if (pthread_create(&uart_thread, NULL, uart_recv_thread, NULL) != 0) {
        //LOG_FATAL("创建串口线程失败");
        return -2;
    }
    // if (pthread_create(&web_thread, NULL, web_ctrl_thread, NULL) != 0) {
    //     //LOG_FATAL("创建Web线程失败");
    //     return -3;
    // }

    // 等待线程退出（主线程阻塞）
    pthread_join(uart_thread, NULL);
    //pthread_join(web_thread, NULL);
    // 资源释放
    close(zigbee_fd);
    // if (mqtt_client != NULL) {
    //     mosquitto_disconnect(mqtt_client);
    //     mosquitto_destroy(mqtt_client);
    //     mosquitto_lib_cleanup();
    // }
    return 0;
}