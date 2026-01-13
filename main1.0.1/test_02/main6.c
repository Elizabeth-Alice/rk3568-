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

int zigbee_fd = -1;
#define ZIGBEE_DEV "/dev/ttyS4"
#define RECV_TIMEOUT 500  // 新增：超时时间 500ms，可根据需求调整



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
    }else if (strcmp(driver->valuestring, "infrared") == 0) {
        // 红外传感器解析
        cJSON *infrared_state = cJSON_GetObjectItem(data, "infrared_state");
        if (infrared_state) {
            dev_state.infrared_state = infrared_state->valueint;
            printf("[红外-ID:%d] 状态: %d", id->valueint, infrared_state->valueint);
        }
    }else if (strcmp(driver->valuestring, "light_sensor") == 0) {
        // 光敏传感器解析
        cJSON *light_intensity = cJSON_GetObjectItem(data, "light_intensity");
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
        
        // 2. 处理根路径请求，返回main3.1.1.html页面
        } else if (strcmp(uri_buf, "/") == 0) {
            printf("处理根路径请求: /\n");
            // 读取HTML文件
            char* html_content = read_html_file("main3.1.1.html");
            if (!html_content) {
                printf("HTML文件读取失败，返回错误页面\n");
                // 如果文件读取失败，返回一个简单的错误页面
                const char* error_html = 
                    "<html><body><h1>错误：无法加载HTML页面</h1>"
                    "<p>请确保main3.1.1.html文件在/aiot-38目录下</p>"
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
    printf("使用/aiot-38/main3.1.1.html作为Web界面\n");

    // 事件循环（阻塞，处理HTTP请求）
    while (1) {
        mg_mgr_poll(&mgr, 1000);  // 每次轮询阻塞1秒
    }

    // 清理资源（实际不会执行，主线程退出时销毁）
    mg_mgr_free(&mgr);
    return 0;
}
    
int main(int argc, char *argv[]) {
    // 初始化Zigbee串口
    if (zigbee_uart_init() < 0) {
        //LOG_FATAL("Zigbee串口初始化失败");
        return -1;
    }
    
    // 创建线程
    pthread_t uart_thread, web_thread, mqtt_thread;
    if (pthread_create(&uart_thread, NULL, uart_recv_thread, NULL) != 0) {
        //LOG_FATAL("创建串口线程失败");
        return -2;
    }
    if (pthread_create(&web_thread, NULL, web_server_thread, NULL) != 0) {
        printf("创建Web线程失败\n");
        return -3;
    }

    // 等待线程退出（主线程阻塞）
    pthread_join(uart_thread, NULL);
    //pthread_join(web_thread, NULL);
    pthread_join(web_thread, NULL);
    // 资源释放
    close(zigbee_fd);
    
    return 0;
}