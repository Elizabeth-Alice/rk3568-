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
#include "mongoose.h"
#include <sys/wait.h>  // 新增：用于进程管理
#include <sys/stat.h>  // 新增：用于文件检查
#include <signal.h>    // 新增：用于信号处理

int zigbee_fd = -1;
#define ZIGBEE_DEV "/dev/ttyS4"
#define RECV_TIMEOUT 500  // 新增：超时时间 500ms，可根据需求调整

// 新增：Web服务端口定义
#define WEB_PORT "8080"

// 新增：视频流服务相关定义
#define VIDEO_DEVICE "/dev/video9"
#define VIDEO_STREAM_PORT "8081"
pid_t video_stream_pid = -1;  // 视频流进程PID

// 新增：函数声明
int start_video_stream(void);
void stop_video_stream(void);
void cleanup_video_stream(void);
int take_snapshot(const char* output_path);  // 新增：拍照函数声明

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
    int infrared_state;      // 红外传感器状态：0-无检测，1-有检测
    int light_intensity;     // 光敏传感器光照强度：0-100
    int thermistor_value;    // 热敏传感器值：0-100
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
    }else if (strcmp(driver->valuestring, "led_pwm") == 0) {
        cJSON *led_state = cJSON_GetObjectItem(data, "led_state");
        cJSON *brightness = cJSON_GetObjectItem(data, "led_brightness");
        if (led_state && brightness) {
            dev_state.led_state = led_state->valueint;
            dev_state.led_brightness = brightness->valueint;
            printf("[LED-ID:%d] 状态: %d, 亮度: %d%%", id->valueint, led_state->valueint, brightness->valueint);
        }
    }else if (strcmp(driver->valuestring, "fan_pwm") == 0) {
        // 新增：风扇状态解析
        cJSON *fan_state = cJSON_GetObjectItem(data, "fan_state");
        cJSON *fan_speed = cJSON_GetObjectItem(data, "fan_speed");
        cJSON *fan_dir = cJSON_GetObjectItem(data, "fan_dir");
        if (fan_state && fan_speed && fan_dir) {
            dev_state.fan_state = fan_state->valueint;
            dev_state.fan_speed = fan_speed->valueint;
            dev_state.fan_dir = fan_dir->valueint;
            printf("[风扇-ID:%d] 状态: %d, 风速: %d%%, 方向: %d", id->valueint, fan_state->valueint, fan_speed->valueint, fan_dir->valueint);
        }
    }else if (strcmp(driver->valuestring, "infrared") == 0) {
        // 红外传感器解析
        cJSON *infrared_state = cJSON_GetObjectItem(data, "infrared_state");
        if (infrared_state) {
            dev_state.infrared_state = infrared_state->valueint;
            printf("[红外-ID:%d] 状态: %d", id->valueint, infrared_state->valueint);
        }
    }else if (strcmp(driver->valuestring, "light") == 0) {
        // 光敏传感器解析
        cJSON *light_intensity = cJSON_GetObjectItem(data, "light");
        if (light_intensity) {
            dev_state.light_intensity = light_intensity->valueint;
            printf("[光敏-ID:%d] 光照强度: %d%%", id->valueint, light_intensity->valueint);
        }
    }else if (strcmp(driver->valuestring, "thermistor") == 0) {
        // 热敏传感器解析
        cJSON *thermistor_value = cJSON_GetObjectItem(data, "thermistor_value");
        if (thermistor_value) {
            dev_state.thermistor_value = thermistor_value->valueint;
            printf("[热敏-ID:%d] 热敏值: %d", id->valueint, thermistor_value->valueint);
        }
    }
    //pthread_mutex_unlock(&zigbee_mutex);
    // printf("[%s-ID:%d] 温湿度数据已转发MQTT → %s\n", driver->valuestring, id->valueint, MQTT_TOPIC_UP_SENSOR);
    //mqtt_publish_msg(MQTT_TOPIC_UP_SENSOR, json_buf);
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
        pthread_mutex_unlock(&zigbee_mutex);
        usleep(10000); // 短延时，降低CPU占用
    }
    //close(zigbee_fd);
    return 0;
}

// ========== 新增：Mongoose Web服务相关代码 ==========
#define WEB_PORT "8080"  // Web服务端口

// 读取HTML文件内容
static char* read_html_file(const char* filename) {
    // 使用绝对路径
char full_path[1024];
    snprintf(full_path, sizeof(full_path), "/aiot-38/%s", filename);
    
    printf("尝试打开HTML文件: %s\n", full_path);
    
    FILE* file = fopen(full_path, "r");
    if (!file) {
        printf("无法打开HTML文件: %s\n", full_path);
        return NULL;
    }
    
    // 获取文件大小
    fseek(file, 0, SEEK_END);
    long size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    if (size <= 0) {
        printf("HTML文件大小为0或无效\n");
        fclose(file);
        return NULL;
    }
    
    // 分配内存并读取文件内容
    char* content = (char*)malloc(size + 1);
    if (!content) {
        printf("内存分配失败\n");
        fclose(file);
        return NULL;
    }
    
    size_t read_size = fread(content, 1, size, file);
content[read_size] = '\0';
    fclose(file);
    
    printf("成功读取HTML文件，大小: %ld 字节\n", size);
    return content;
}

// LED控制函数 - 通过串口发送LED控制命令（符合设备协议）
int send_led_control(int led_state, int led_brightness) {
    pthread_mutex_lock(&zigbee_mutex);
    
    // 获取当前时间戳
    time_t current_time = time(NULL);
    
    // 构建LED控制JSON命令（符合设备协议）
    cJSON *root = cJSON_CreateObject();
cJSON_AddStringToObject(root, "source", "rk3568");  // 源设备
    cJSON_AddStringToObject(root, "driver", "led_pwm"); // 驱动类型
    cJSON_AddNumberToObject(root, "id", 6);             // 设备ID
    
    cJSON *data = cJSON_CreateObject();
    cJSON_AddNumberToObject(data, "data_time", current_time);  // 时间戳
    cJSON_AddNumberToObject(data, "led_id", 1);                 // LED ID
    cJSON_AddNumberToObject(data, "led_state", led_state);      // LED状态
    cJSON_AddNumberToObject(data, "led_brightness", led_brightness); // LED亮度
    cJSON_AddItemToObject(root, "data", data);
    
    // 转换为JSON字符串
    char *json_str = cJSON_PrintUnformatted(root);
    printf("LED控制JSON命令: %s\n", json_str);
    cJSON_Delete(root);
    
    // 通过串口发送命令
    int result = write(zigbee_fd, json_str, strlen(json_str));
    // 释放JSON字符串内存
    free(json_str);
    if (result < 0) {
        perror("LED控制命令发送失败");
        pthread_mutex_unlock(&zigbee_mutex);
        return -1;
    }
    
    // 更新设备状态
    dev_state.led_state = led_state;
    dev_state.led_brightness = led_brightness;
    
    pthread_mutex_unlock(&zigbee_mutex);
printf("LED控制命令发送成功: 状态=%d, 亮度=%d%%\n", led_state, led_brightness);
    return 0;
}

// 新增：风扇控制函数 - 通过串口发送风扇控制命令（符合设备协议）
int send_fan_control(int fan_state, int fan_speed, int fan_dir) {
    pthread_mutex_lock(&zigbee_mutex);
    
    // 获取当前时间戳
    time_t current_time = time(NULL);
    
    // 构建风扇控制JSON命令（符合设备协议）
    cJSON *root = cJSON_CreateObject();
cJSON_AddStringToObject(root, "source", "rk3568");  // 源设备
    cJSON_AddStringToObject(root, "driver", "fan_pwm"); // 驱动类型
    cJSON_AddNumberToObject(root, "id", 7);             // 设备ID
    
    cJSON *data = cJSON_CreateObject();
    cJSON_AddNumberToObject(data, "data_time", current_time);  // 时间戳
    cJSON_AddNumberToObject(data, "fan_id", 1);                 // 风扇 ID
    cJSON_AddNumberToObject(data, "fan_state", fan_state);      // 风扇状态
    cJSON_AddNumberToObject(data, "fan_speed", fan_speed);      // 风扇风速
    cJSON_AddNumberToObject(data, "fan_dir", fan_dir);          // 风扇方向
    cJSON_AddItemToObject(root, "data", data);
    
    // 转换为JSON字符串
    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    
    // 通过串口发送命令
    int result = write(zigbee_fd, json_str, strlen(json_str));
    // 释放JSON字符串内存
    free(json_str);
    if (result < 0) {
        perror("风扇控制命令发送失败");
        pthread_mutex_unlock(&zigbee_mutex);
        return -1;
    }
    
    // 更新设备状态
    dev_state.fan_state = fan_state;
    dev_state.fan_speed = fan_speed;
    dev_state.fan_dir = fan_dir;
    
    pthread_mutex_unlock(&zigbee_mutex);
    printf("风扇控制命令发送成功: 状态=%d, 风速=%d%%, 方向=%d\n", fan_state, fan_speed, fan_dir);
    return 0;
}

// Mongoose事件处理回调函数（处理HTTP请求）- 适配新版本API
static void mg_web_event_handler(struct mg_connection *c, int ev, void *ev_data) {
    if (ev == MG_EV_HTTP_REQUEST) {  // 新版本使用MG_EV_HTTP_REQUEST
        struct http_message *hm = (struct http_message *) ev_data;
        
        // 创建URI字符串用于比较
        char uri_buf[256];
        int uri_len = hm->uri.len < sizeof(uri_buf) - 1 ? hm->uri.len : sizeof(uri_buf) - 1;
        strncpy(uri_buf, hm->uri.p, uri_len);
        uri_buf[uri_len] = '\0';
        
        printf("收到HTTP请求: URI=%s (长度=%d)\n", uri_buf, (int)hm->uri.len);
        
        // 1. 处理设备数据API请求：/api/device_state
        if (strcmp(uri_buf, "/api/device_state") == 0) {
            printf("处理API请求: /api/device_state\n");
            // 加锁读取设备状态
            pthread_mutex_lock(&zigbee_mutex);
            // 构建JSON响应
cJSON *root = cJSON_CreateObject();
            cJSON_AddNumberToObject(root, "led_state", dev_state.led_state);
            cJSON_AddNumberToObject(root, "led_brightness", dev_state.led_brightness);
            cJSON_AddNumberToObject(root, "fan_state", dev_state.fan_state);
            cJSON_AddNumberToObject(root, "fan_speed", dev_state.fan_speed);
            cJSON_AddNumberToObject(root, "fan_dir", dev_state.fan_dir);
            cJSON_AddNumberToObject(root, "temperature", dev_state.temp);
            cJSON_AddNumberToObject(root, "humidity", dev_state.humi);
            cJSON_AddNumberToObject(root, "infrared_state", dev_state.infrared_state);
            cJSON_AddNumberToObject(root, "light_intensity", dev_state.light_intensity);
            cJSON_AddNumberToObject(root, "thermistor_value", dev_state.thermistor_value);
            pthread_mutex_unlock(&zigbee_mutex);

            // 转换为JSON字符串并响应
            char *json_str = cJSON_Print(root);
            cJSON_Delete(root);
            
            // 新版本使用mg_send_head和mg_send
            mg_send_head(c, 200, strlen(json_str), 
                "Content-Type: application/json; charset=utf-8\r\n"
                "Access-Control-Allow-Origin: *");
            mg_send(c, json_str, strlen(json_str));
            free(json_str);  // 释放JSON字符串
        
        // 2. 处理LED控制API请求：/api/led_control
        } else if (strcmp(uri_buf, "/api/led_control") == 0) {
            printf("处理API请求: /api/led_control\n");
            
            // 解析POST参数
            char query_buf[256];
            int query_len = hm->query_string.len < sizeof(query_buf) - 1 ? hm->query_string.len : sizeof(query_buf) - 1;
            strncpy(query_buf, hm->query_string.p, query_len);
            query_buf[query_len] = '\0';
            
            printf("查询参数: %s\n", query_buf);
            
            // 解析参数
            int led_state = -1;
            int led_brightness = -1;
            
            char *token = strtok(query_buf, "&");
            while (token != NULL) {
                if (strncmp(token, "state=", 6) == 0) {
                    led_state = atoi(token + 6);
                } else if (strncmp(token, "brightness=", 11) == 0) {
                    led_brightness = atoi(token + 11);
                }
                token = strtok(NULL, "&");
            }
            
            // 验证参数
            if (led_state == -1 && led_brightness == -1) {
                const char *error = "缺少参数: state 或 brightness";
mg_send_head(c, 400, strlen(error), "");
                mg_send(c, error, strlen(error));
                return;
            }
            
            // 如果只改变亮度，保持当前状态；如果改变状态，使用当前亮度或默认值
            if (led_state == -1) {
                led_state = dev_state.led_state;  // 保持当前状态
            }
            if (led_brightness == -1) {
                led_brightness = dev_state.led_brightness;  // 保持当前亮度
            }
            
            // 发送LED控制命令
            int result = send_led_control(led_state, led_brightness);
            
            // 构建响应
            cJSON *response = cJSON_CreateObject();
            if (result == 0) {
                cJSON_AddStringToObject(response, "status", "success");
                cJSON_AddNumberToObject(response, "led_state", led_state);
                cJSON_AddNumberToObject(response, "led_brightness", led_brightness);
            } else {
                cJSON_AddStringToObject(response, "status", "error");
                cJSON_AddStringToObject(response, "message", "LED控制失败");
            }
            
            char *json_str = cJSON_Print(response);
            cJSON_Delete(response);
mg_send_head(c, result == 0 ? 200 : 500, strlen(json_str), 
                "Content-Type: application/json; charset=utf-8\r\n"
                "Access-Control-Allow-Origin: *");
            mg_send(c, json_str, strlen(json_str));
            free(json_str);
        
        // 3. 新增：处理风扇控制API请求：/api/fan_control
        } else if (strcmp(uri_buf, "/api/fan_control") == 0) {
            printf("处理API请求: /api/fan_control\n");
            
            // 解析POST参数
            char query_buf[256];
            int query_len = hm->query_string.len < sizeof(query_buf) - 1 ? hm->query_string.len : sizeof(query_buf) - 1;
            strncpy(query_buf, hm->query_string.p, query_len);
            query_buf[query_len] = '\0';
            
            printf("查询参数: %s\n", query_buf);
            
            // 解析参数
            int fan_state = -1;
            int fan_speed = -1;
            int fan_dir = -1;
            
            char *token = strtok(query_buf, "&");
            while (token != NULL) {
                if (strncmp(token, "state=", 6) == 0) {
                    fan_state = atoi(token + 6);
                } else if (strncmp(token, "speed=", 6) == 0) {
                    fan_speed = atoi(token + 6);
                } else if (strncmp(token, "direction=", 10) == 0) {
                    fan_dir = atoi(token + 10);
                }
                token = strtok(NULL, "&");
            }
            
            // 验证参数
            if (fan_state == -1 && fan_speed == -1 && fan_dir == -1) {
                const char *error = "缺少参数: state、speed 或 direction";
                mg_send_head(c, 400, strlen(error), "");
                mg_send(c, error, strlen(error));
                return;
            }
            
            // 如果只改变速度或方向，保持当前状态；如果改变状态，使用当前速度或默认值
            if (fan_state == -1) {
                fan_state = dev_state.fan_state;  // 保持当前状态
            }
            if (fan_speed == -1) {
                fan_speed = dev_state.fan_speed;  // 保持当前速度
            }
            if (fan_dir == -1) {
                fan_dir = dev_state.fan_dir;      // 保持当前方向
            }
            
            // 发送风扇控制命令
            int result = send_fan_control(fan_state, fan_speed, fan_dir);
            
            // 构建响应
            cJSON *response = cJSON_CreateObject();
            if (result == 0) {
                cJSON_AddStringToObject(response, "status", "success");
                cJSON_AddNumberToObject(response, "fan_state", fan_state);
                cJSON_AddNumberToObject(response, "fan_speed", fan_speed);
                cJSON_AddNumberToObject(response, "fan_dir", fan_dir);
            } else {
                cJSON_AddStringToObject(response, "status", "error");
                cJSON_AddStringToObject(response, "message", "风扇控制失败");
            }
            
            char *json_str = cJSON_Print(response);
            cJSON_Delete(response);
            
            mg_send_head(c, result == 0 ? 200 : 500, strlen(json_str), 
                "Content-Type: application/json; charset=utf-8\r\n"
                "Access-Control-Allow-Origin: *");
            mg_send(c, json_str, strlen(json_str));
            free(json_str);
        
        // 4. 处理视频流代理请求：/video_stream
        } else if (strncmp(uri_buf, "/video_stream", 13) == 0) {
            printf("视频流代理已禁用，请前端直接访问端口%s\n", VIDEO_STREAM_PORT);
            const char *message = "视频流服务已直接运行在8081端口，请前端直接访问";
            mg_send_head(c, 200, strlen(message), "Content-Type: text/plain; charset=utf-8");
            mg_send(c, message, strlen(message));
        
        // 5. 新增：处理拍照API请求：/api/snapshot
        } else if (strcmp(uri_buf, "/api/snapshot") == 0) {
            printf("处理API请求: /api/snapshot\n");
            
            // 方案1：使用视频流服务的快照功能
            char temp_file[] = "/tmp/temp_snapshot.jpg";
            char curl_cmd[512];
            int result = -1;
            
            // 构建curl命令获取快照
            snprintf(curl_cmd, sizeof(curl_cmd), 
                    "curl -s --connect-timeout 5 -o %s http://localhost:%s/?action=snapshot 2>/dev/null", 
                    temp_file, VIDEO_STREAM_PORT);
            
            printf("尝试通过视频流服务获取快照: %s\n", curl_cmd);
            
            // 执行curl命令
            result = system(curl_cmd);
            
            if (result == 0) {
                // 检查文件是否创建成功
                struct stat st;
                if (stat(temp_file, &st) == 0 && st.st_size > 0) {
                    // 读取图片文件
                    FILE* image_file = fopen(temp_file, "rb");
                    if (image_file) {
                        // 获取文件大小
                        fseek(image_file, 0, SEEK_END);
                        long file_size = ftell(image_file);
                        fseek(image_file, 0, SEEK_SET);
                        
                        if (file_size > 0) {
                            // 分配内存并读取文件
                            char* image_data = (char*)malloc(file_size);
                            if (image_data) {
                                size_t read_size = fread(image_data, 1, file_size, image_file);
                                
                                if (read_size == file_size) {
                                    // 返回图片数据
                                    mg_send_head(c, 200, file_size, 
                                        "Content-Type: image/jpeg\r\n"
                                        "Content-Disposition: inline; filename=\"snapshot.jpg\"\r\n"
                                        "Access-Control-Allow-Origin: *");
                                    mg_send(c, image_data, file_size);
                                    printf("拍照API响应成功: 通过视频流服务获取快照，文件大小 %ld 字节\n", file_size);
                                } else {
                                    const char* error = "图片文件读取不完整";
                                    mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                                    mg_send(c, error, strlen(error));
                                    printf("拍照API错误: %s\n", error);
                                }
                                
                                free(image_data);
                            } else {
                                const char* error = "内存分配失败";
                                mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                                mg_send(c, error, strlen(error));
                                printf("拍照API错误: %s\n", error);
                            }
                        } else {
                            const char* error = "图片文件大小为0";
                            mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                            mg_send(c, error, strlen(error));
                            printf("拍照API错误: %s\n", error);
                        }
                        
                        fclose(image_file);
                    } else {
                        const char* error = "无法打开临时图片文件";
                        mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                        mg_send(c, error, strlen(error));
                        printf("拍照API错误: %s\n", error);
                    }
                    
                    // 删除临时文件
                    remove(temp_file);
                } else {
                    const char* error = "快照获取失败，请检查视频流服务";
                    mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                    mg_send(c, error, strlen(error));
                    printf("拍照API错误: %s\n", error);
                }
            } else {
                // 方案1失败，尝试方案2：停止视频流服务后直接拍照
                printf("视频流服务快照失败，尝试直接拍照...\n");
                
                // 停止视频流服务
                stop_video_stream();
                
                // 等待设备释放
                sleep(2);
                
                // 生成唯一的文件名
                char filename[256];
                time_t now = time(NULL);
                struct tm *tm_info = localtime(&now);
                strftime(filename, sizeof(filename), "/tmp/snapshots/snapshot_%Y%m%d_%H%M%S.jpg", tm_info);
                
                // 执行直接拍照
                int direct_result = take_snapshot(filename);
                
                if (direct_result == 0) {
                    // 读取图片文件
                    FILE* image_file = fopen(filename, "rb");
                    if (image_file) {
                        fseek(image_file, 0, SEEK_END);
                        long file_size = ftell(image_file);
                        fseek(image_file, 0, SEEK_SET);
                        
                        if (file_size > 0) {
                            char* image_data = (char*)malloc(file_size);
                            if (image_data) {
                                size_t read_size = fread(image_data, 1, file_size, image_file);
                                
                                if (read_size == file_size) {
                                    mg_send_head(c, 200, file_size, 
                                        "Content-Type: image/jpeg\r\n"
                                        "Content-Disposition: inline; filename=\"snapshot.jpg\"\r\n"
                                        "Access-Control-Allow-Origin: *");
                                    mg_send(c, image_data, file_size);
                                    printf("拍照API响应成功: 直接拍照，文件大小 %ld 字节\n", file_size);
                                } else {
                                    const char* error = "图片文件读取不完整";
                                    mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                                    mg_send(c, error, strlen(error));
                                    printf("拍照API错误: %s\n", error);
                                }
                                free(image_data);
                            } else {
                                const char* error = "内存分配失败";
                                mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                                mg_send(c, error, strlen(error));
                                printf("拍照API错误: %s\n", error);
                            }
                        } else {
                            const char* error = "图片文件大小为0";
                            mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                            mg_send(c, error, strlen(error));
                            printf("拍照API错误: %s\n", error);
                        }
                        fclose(image_file);
                    } else {
                        const char* error = "无法打开图片文件";
                        mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                        mg_send(c, error, strlen(error));
                        printf("拍照API错误: %s\n", error);
                    }
                } else {
                    const char* error = "直接拍照失败，请检查视频设备";
                    mg_send_head(c, 500, strlen(error), "Content-Type: text/plain");
                    mg_send(c, error, strlen(error));
                    printf("拍照API错误: %s\n", error);
                }
                
                // 重新启动视频流服务
                printf("重新启动视频流服务...\n");
                start_video_stream();
            }
        
        // 6. 处理根路径请求，返回main3.1.1.html页面
        } else if (strcmp(uri_buf, "/") == 0) {
            printf("处理根路径请求: /\n");
            // 读取HTML文件
            char* html_content = read_html_file("main3.1.1.html");
            if (!html_content) {
                printf("HTML文件读取失败，返回错误页面\n");
                // 修改：更新错误提示信息
                const char* error_html = 
                    "<html><body><h1>错误：无法加载HTML页面</h1>"
                    "<p>请确保main3.1.1.html文件在当前目录下</p>"
                    "<p>当前程序运行目录：</p>"
                    "<pre>";
                
                // 添加当前目录列表
                char cmd[1024];
                snprintf(cmd, sizeof(cmd), "pwd && ls -la");
                FILE* ls_output = popen(cmd, "r");
                if (ls_output) {
                    char buffer[1024];
                    char* temp_html = strdup(error_html);
                    while (fgets(buffer, sizeof(buffer), ls_output)) {
                        // 动态构建错误页面
                        char* new_temp = (char*)malloc(strlen(temp_html) + strlen(buffer) + 1);
                        strcpy(new_temp, temp_html);
                        strcat(new_temp, buffer);
                        free(temp_html);
                        temp_html = new_temp;
                    }
                    pclose(ls_output);
                    
                    char* final_html = (char*)malloc(strlen(temp_html) + 100);
                    strcpy(final_html, temp_html);
                    strcat(final_html, "</pre></body></html>");
                    free(temp_html);
                    
                    mg_send_head(c, 500, strlen(final_html), "Content-Type: text/html; charset=utf-8");
                    mg_send(c, final_html, strlen(final_html));
                    free(final_html);
                } else {
                    const char* simple_error = "<html><body><h1>错误：无法加载HTML页面</h1></body></html>";
                    mg_send_head(c, 500, strlen(simple_error), "Content-Type: text/html; charset=utf-8");
                    mg_send(c, simple_error, strlen(simple_error));
                }
                return;
            }
            
            printf("成功读取HTML文件，返回页面内容\n");
            // 返回HTML页面
            mg_send_head(c, 200, strlen(html_content), "Content-Type: text/html; charset=utf-8");
            mg_send(c, html_content, strlen(html_content));
            free(html_content);  // 释放内存
        
        // 3. 其他路径返回404
        } else {
            printf("未知路径请求: %s，返回404\n", uri_buf);
            const char *not_found = "404 Not Found";
            mg_send_head(c, 404, strlen(not_found), "");
            mg_send(c, not_found, strlen(not_found));
        }
    }
}

// Mongoose Web服务线程入口
void* web_server_thread(void* arg) {
    struct mg_mgr mgr;        // Mongoose管理器
    struct mg_connection *c;  // Mongoose连接

    // 初始化Mongoose（新版本需要传入user_data参数）
    mg_mgr_init(&mgr, NULL);
    
    // 启动HTTP服务（监听8080端口）- 使用mg_bind
    c = mg_bind(&mgr, "0.0.0.0:" WEB_PORT, mg_web_event_handler);
    if (c == NULL) {
        printf("Mongoose Web服务启动失败（端口%s）\n", WEB_PORT);
        pthread_exit((void*)-1);
    }
    
    // 设置HTTP协议处理
    mg_set_protocol_http_websocket(c);
    printf("Mongoose Web服务已启动：http://本机IP:%s\n", WEB_PORT);
    // 修改：更新提示信息
    printf("使用当前目录下的main3.1.1.html作为Web界面\n");

    // 事件循环（阻塞，处理HTTP请求）
    while (1) {
        mg_mgr_poll(&mgr, 1000);  // 每次轮询阻塞1秒
    }

    // 清理资源（实际不会执行，主线程退出时销毁）
    mg_mgr_free(&mgr);
    return 0;
}

int main(int argc, char *argv[]) {
    // 注册退出清理函数
    atexit(cleanup_video_stream);
    
    printf("=== RK3568 智能物联网控制中心启动 ===\n");
    
    // 启动视频流服务
    printf("1. 启动视频流服务...\n");
    int video_result = start_video_stream();
    if (video_result < 0) {
        printf("视频流服务启动失败，错误码: %d\n", video_result);
        // 继续启动其他服务，视频流服务非必需
    }
    
    // 初始化Zigbee串口
    printf("2. 初始化Zigbee串口...\n");
    if (zigbee_uart_init() < 0) {
        printf("Zigbee串口初始化失败\n");
        cleanup_video_stream();
        return -1;
    }
    
    // 创建线程
    printf("3. 创建服务线程...\n");
    pthread_t uart_thread, web_thread;
    
    if (pthread_create(&uart_thread, NULL, uart_recv_thread, NULL) != 0) {
        printf("创建串口线程失败\n");
        cleanup_video_stream();
        return -2;
    }
    
    if (pthread_create(&web_thread, NULL, web_server_thread, NULL) != 0) {
        printf("创建Web线程失败\n");
        cleanup_video_stream();
        return -3;
    }
    
    printf("=== 所有服务启动完成 ===\n");
    printf("Web服务: http://本机IP:%s\n", WEB_PORT);
    if (video_result >= 0) {
        printf("视频流: http://本机IP:%s/?action=stream (前端直接访问)\n", VIDEO_STREAM_PORT);
        printf("快照: http://本机IP:%s/?action=snapshot\n", VIDEO_STREAM_PORT);
    }
    printf("按Ctrl+C退出程序...\n");

    // 等待线程退出（主线程阻塞）
    pthread_join(uart_thread, NULL);
    pthread_join(web_thread, NULL);
    
    // 资源释放
    close(zigbee_fd);
    
    return 0;
}

// ... 新增：视频流服务管理函数 ==========

// 删除重复的read_html_file函数定义，保留第268行的版本

// 检查设备是否存在
int check_video_device(const char* device_path) {
    struct stat st;
    if (stat(device_path, &st) == 0) {
        printf("视频设备 %s 存在\n", device_path);
        return 1;
    } else {
        printf("视频设备 %s 不存在\n", device_path);
        return 0;
    }
}

// 启动视频流服务
int start_video_stream(void) {
    // 检查视频设备是否存在
    if (!check_video_device(VIDEO_DEVICE)) {
        printf("错误：视频设备 %s 不存在，无法启动视频流服务\n", VIDEO_DEVICE);
        return -1;
    }
    
    // 检查mjpg-streamer是否可用
    if (system("which mjpg_streamer > /dev/null 2>&1") != 0) {
        printf("错误：mjpg-streamer未安装或不在PATH中\n");
        return -2;
    }
    
    // 创建子进程启动视频流服务
    pid_t pid = fork();
    if (pid < 0) {
        perror("fork失败");
        return -3;
    }
    
    if (pid == 0) {
        // 子进程：启动mjpg-streamer
        printf("启动视频流服务...\n");
        printf("设备: %s, 端口: %s\n", VIDEO_DEVICE, VIDEO_STREAM_PORT);
        
        // 修改：添加分辨率参数避免警告
        char command[512];
        snprintf(command, sizeof(command), 
                "mjpg_streamer -i \"/usr/lib/input_uvc.so -n -d %s -r 640x360\" -o \"/usr/lib/output_http.so -p %s -w /usr/share/mjpg-streamer/www\"",
                VIDEO_DEVICE, VIDEO_STREAM_PORT);
        
        printf("执行命令: %s\n", command);
        
        // 执行命令
        execl("/bin/sh", "sh", "-c", command, NULL);
        
        // 如果执行失败
        perror("execl失败");
        exit(1);
    } else {
        // 父进程：记录PID
        video_stream_pid = pid;
        printf("视频流服务已启动，PID: %d\n", pid);
        
        // 等待3秒让服务启动（增加等待时间）
        sleep(3);
        
        // 修改：使用更可靠的检查方法
        char check_cmd[256];
        snprintf(check_cmd, sizeof(check_cmd), "curl -s --connect-timeout 3 http://localhost:%s/?action=stream > /dev/null 2>&1", VIDEO_STREAM_PORT);
        
        if (system(check_cmd) == 0) {
            printf("视频流服务运行正常，访问地址: http://本机IP:%s/?action=stream\n", VIDEO_STREAM_PORT);
            return 0;
        } else {
            printf("警告：视频流服务可能未正确启动，但将继续运行\n");
            printf("可以尝试手动访问: http://localhost:%s/?action=stream\n", VIDEO_STREAM_PORT);
            return 1;  // 返回1而不是错误，让程序继续运行
        }
    }
}

// 停止视频流服务
void stop_video_stream(void) {
    if (video_stream_pid > 0) {
        printf("停止视频流服务 (PID: %d)...\n", video_stream_pid);
        
        // 发送SIGTERM信号
        kill(video_stream_pid, SIGTERM);
        
        // 等待进程结束
        int status;
        waitpid(video_stream_pid, &status, 0);
        
        video_stream_pid = -1;
        printf("视频流服务已停止\n");
    }
}

// 清理视频流服务（程序退出时调用）
void cleanup_video_stream(void) {
    stop_video_stream();
}

// 新增：拍照函数 - 使用v4l2-ctl命令拍照
int take_snapshot(const char* output_path) {
    char command[1024];
    
    // 创建目录（如果不存在）
    char dir_cmd[512];
    snprintf(dir_cmd, sizeof(dir_cmd), "mkdir -p /tmp/snapshots");
    system(dir_cmd);
    
    // 构建v4l2-ctl拍照命令
    snprintf(command, sizeof(command), 
             "v4l2-ctl --device=%s --set-fmt-video=width=640,height=480,pixelformat=MJPG --stream-mmap --stream-to=%s --stream-count=1 2>/dev/null", 
             VIDEO_DEVICE, output_path);
    
    printf("执行拍照命令: %s\n", command);
    
    int result = system(command);
    if (result == 0) {
        // 检查文件是否创建成功
        struct stat st;
        if (stat(output_path, &st) == 0 && st.st_size > 0) {
            printf("拍照成功: %s (大小: %ld 字节)\n", output_path, st.st_size);
            return 0;
        } else {
            printf("拍照失败: 文件创建失败\n");
            return -1;
        }
    } else {
        printf("拍照失败: 命令执行错误 (返回值: %d)\n", result);
        return -1;
    }
}