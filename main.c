#include "stm32f10x.h"
#include "./uart/bsp_uart.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "./pwm/bsp_pwm.h"
#include "./i2c/bsp_i2c_gpio.h"
#include "./i2c/bsp_i2c_ee.h"

#define CR '\r'
#define LF '\n'
#define CRLF "\r\n"

#define COMMAND_MAX_LENGTH 128
enum CMD_ID{
    CMD_NULL,
    HELP = 1,
    INFO,
    DUMP,
    I2CREAD,
    I2CWRITE,
    BLSETBRIGHTNESS,
    BLSETRATIOS,
    BLSWITCH,
    BLSETTINGSGET,
    BLSET2DCURRENT,
    BLSET3DCURRENT,
    BLSETPWM,
    SET2DCTRLMODE,
    POKE32,
    PEEK32,
    POKE16,
    PEEK16,
    POKE8,
    PEEK8,
    GPIOREAD,
    GPIOREADPIN,
    GPIOSETPIN,
    GPIOCLRPIN,
    BLFACTORYRESET,
    UNKNOWCMD = 0xff,
};

// messages
#define UNKNOWN_COMMAND  "[ERROR]unknown command"
#define COMMAND_TOO_LONG "[ERROR]command too long"
#define WRONG_FORMAT     "[ERROR]wrong fomart,see HELP"

//内部flash写入的起始地址与结束地址
#define WRITE_START_ADDR  ((uint32_t)0x08008000)
#define WRITE_END_ADDR    ((uint32_t)0x0800C000)

#define FLASH_PAGE_SIZE   ((uint16_t)0x400) //1024
#define PAGE_ADDR(n)      (0x08000000 + n*1024)

#define DUTY_MAX_3D 2785   // 2785/4096 = 68% ,which is PWM 3D's LIMIT

//tokens for cmd analysis
char *g_tokens[5];
char board_info[128] = "version: 1.0";
char cmd_buffer[COMMAND_MAX_LENGTH];
char g_usart_buf[COMMAND_MAX_LENGTH];
int cmd_received = 0;
unsigned short usart_buf_length=0;
char cmd_help_str[1024] =   "HELP -- Display all commands\r\n" \
                            "INFO -- Display Board Information\r\n" \
                            "DUMP -- Dump 8556 register\r\n" \
                            "I2CREAD  <uint8_addr>               -- Read 8556 reg with i2c\r\n" \
                            "I2CWRITE <uint8_addr> <uint8_value> -- Write 8556 reg with i2c\r\n" \
                            "BLSETBRIGHTNESS  <Brightness 0-255> -- default 102\r\n" \
                            "BLSETRATIOS <Mode 2 or 3>  <RATIO_2D 0.00-1.00>  <RATIO_3D 0.00-3.00>\r\n" \
                            "BLSWITCH    <Mode 2 or 3>\r\n" \
                            "SET2DCTRLMODE <Mode 0 or 1> -- 2D control mode, 0 for i2c, 1 for pwm\r\n" \
                            "BLSETPWM    <2 or 3> <Duty cycle 0-4096> --0~4096 map to 1% ~ 100%\r\n" \
                            "BLSET2DCURRENT  <0.00-25.00mA>\r\n" \
                            "BLSET3DCURRENT  <0.00-14.00mA>\r\n" \
                            "BLSETTINGSGET\r\n" \
                            "POKE32 <Address> <Data>\r\n" \
                            "POKE16 <Address> <Data>\r\n" \
                            "POKE8  <Address> <Data>\r\n" \
                            "PEEK32 <Address>\r\n" \
                            "PEEK16 <Address>\r\n" \
                            "PEEK8  <Address>\r\n" \
                            "GPIOREAD    <port>\r\n" \
                            "GPIOREADPIN <port> <pin>\r\n" \
                            "GPIOSETPIN  <port> <pin>\r\n" \
                            "GPIOCLRPIN  <port> <pin>\r\n" \
                            "BLFACTORYRESET -- Reset to Factory Defaults.\r\n";

#define I2C_CTRL 0
#define PWM_CTRL 1

#define mode_index 0
#define brightness_index 1
#define mode2ratio2d_index 2
#define mode2ratio3d_index 3
#define mode3ratio2d_index 4
#define mode3ratio3d_index 5
#define ctrlmode2d_index 6
#define STORE_LEN 7

/*
  These values will be saved in stm32's flash PAGE_ADDR(50).
  [0]mode, 
  [1]brightness, 
  [2]mode2ratio2d(0.00~1.00,default:1.0,map to 1~100 in memory), 
  [3]mode2ratio3d(0.00~3.00,default:0.15,map to 1~300 in memory), 
  [4]mode3ratio2d(0.00~1.00,default:0,map to 1~100 in memory), 
  [5]mode3ratio3d(0.00~3.00,default:1.0,map to 1~300 in memory),
  [6]ctrlmode(0 for i2c, 1 for pwm)
*/
static uint32_t g_setting[STORE_LEN] = {2,102,100,15,0,100,I2C_CTRL}; 

int cmd_id = CMD_NULL;
int cmd_key_num = 0;
static unsigned char i2c_init_table[][2] = {
    {0xa1, 0x5F},  //hight bit(8~11)
    {0xa0, 0xff},  //low bit(0~7)  25mA
    {0x16, 0x3F},  //5channel LED enable 0x1F
    {0xa9, 0x40},  //VBOOST_MAX = 010,17.9~23.1
    {0x9e, 0x22},  //VBOOST_RANGE = 1
    {0xa2, 0x2b},  //BOOST_FSET_EN PWM_FSET_EN 
    {0xa6, 0x05},  //VBOOST 
    {0x01, 0x05},  //0x01 pwm only set brightness,0x5 I2c set brightness
    {0xff, 0xff},  //ending flag
};


void get_flash_setting(void);
void save_setting_in_flash(void);
void set_flash_tag(void);
void set_sys_brightness(uint8_t brightness);

// send one byte through USART
void usart_send(const char chr) {
    while(!(USART1->SR & USART_SR_TC));
    USART1->DR = chr;
}

// send a string through USART
void usart_send_string(const char *s) {
    int i=0;
    while (s[i]) {
        usart_send(s[i++]);
    }
}

void usart_send_newline(void) {
    usart_send_string(CRLF);
}

void usart_send_line(const char *s) {
    usart_send_string(s);
    usart_send_string(CRLF);
}

void USART1_IRQHandler() {
    unsigned char received;

    if ((USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)) {
        received = USART_ReceiveData(USART1);

        if (received == CR) {
            g_usart_buf[usart_buf_length] = 0;
            //usart_send_newline();
            //command received
            cmd_received = 1;
            usart_buf_length = 0;
        } else if (received == LF) {
            // ignore
        } else {
            if (usart_buf_length == COMMAND_MAX_LENGTH) {
                usart_send_newline();
                usart_send_line(COMMAND_TOO_LONG);
                usart_buf_length = 0;
                return;

            }

            g_usart_buf[usart_buf_length++] = received;

        }
    }
}

void delay(u16 num)
{
  u16 i,j;
  for(i=0;i<num;i++)
    for(j=0;j<0x800;j++);
}

int init_chip_8556()
{
    uint8_t tData[3];
    int i=0, ending_flag=0;
    uint8_t read_data;
    while (ending_flag == 0) 
    {
        if (i2c_init_table[i][0] == 0xff) {    //special mark
                if (i2c_init_table[i][1] == 0xff) { //ending flag
                        ending_flag = 1;
                }
        }
        else {
            tData[0]=i2c_init_table[i][0];
            tData[1]=i2c_init_table[i][1];
            ee_WRITE_BYTES(tData[0], &tData[1], 1);
            delay(400);
            ee_READ_BYTES(tData[0], &read_data, 1);
            if(tData[1] == read_data)
            {
                printf("address:0x%x, value:0x%x,set successfully\r\n",tData[0],read_data);
            }
            else
            {
                printf("address:0x%x, value:0x%x,set fail\r\n",tData[0],read_data);
                return -1;
            }
        }
        i++;
    }
    return 0;
}

void dump_chip_8556()
{
    uint8_t tData[3];
    int i=0, ending_flag=0;
    uint8_t read_data = 0;
    printf("Dump 8556 reg:\r\n");
    printf("address          value\r\n");
    while (ending_flag == 0) 
    {
        if (i2c_init_table[i][0] == 0xff) {    //special mark
            if (i2c_init_table[i][1] == 0xff) { //ending flag
                ending_flag = 1;
            }
        }
        else {
            tData[0]=i2c_init_table[i][0];
            tData[1]=i2c_init_table[i][1];
            ee_READ_BYTES(tData[0], &read_data, 1);
            printf("0x%-15x0x%-5x\r\n",tData[0],read_data);
        }
        i++;
    }
}

int i2c_read_8556()
{
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        return -1;
    }
    char *str = NULL;
    
    uint8_t read_data = 0;
    int address_input = strtol(g_tokens[1],&str,16);
    if(address_input > 255 || address_input < 0 || (*str) != '\0')
    {
        usart_send_line("[ERROR]address can only be a 8 bit HEX num!");
        return -1;
    }
    
    uint8_t address = (uint8_t)address_input;
    ee_READ_BYTES(address, &read_data, 1);
    printf("address          value\r\n");
    printf("0x%-15x0x%-5x\r\n",address,read_data);
    return 0;
}


int i2c_write_8556()
{
    if(cmd_key_num != 3)
    {
        usart_send_line(WRONG_FORMAT);
        return -1;
    }
    char *str = NULL;
    
    int address_input = strtol(g_tokens[1],&str,16);

    if(address_input > 255 || address_input < 0 || (*str) != '\0')
    {
        usart_send_line("[ERROR]address can only be a uint8 HEX num!");
        return -1;
    }
    int value_input = strtol(g_tokens[2],&str,16);
    if(value_input > 255 || value_input < 0 || (*str) != '\0')
    {
        usart_send_line("[ERROR]value can only be a uint8 HEX num!");
        return -1;
    }

    uint8_t read_data = 0;
    uint8_t address = (uint8_t)address_input;
    uint8_t value = (uint8_t)value_input;
    
    ee_WRITE_BYTES(address, &value, 1);
    ee_READ_BYTES(address, &read_data, 1);
    if(value == read_data)
    {
        printf("write successfully,result:\r\n");
        printf("address          value\r\n");
        printf("0x%-15x0x%-5x\r\n",address,read_data);
    }
    else
    {
        printf("The result is not correct!");
        return -1;
    }
    
    return 0;
}

int is_flash_new()
{
    //在page51中存放标志位，根据标志位判断是否设置默认值  
    uint32_t *read_p = (uint32_t *)PAGE_ADDR(51);
    if(*read_p == 0x5a5b5c)
        return 0;
    else
        return 1;
}

void set_flash_tag()
{
    FLASH_Unlock();
    
    FLASH_ErasePage(PAGE_ADDR(51));
    
    uint32_t write_addr = PAGE_ADDR(51);
    uint32_t wrtie_data = 0x5a5b5c;
    
    FLASH_ProgramWord(write_addr,wrtie_data);

    //write board info into flash
    write_addr += 4;
    uint32_t *wrtie_data_p = (uint32_t *)board_info;
    for(int i = 0;i < 32;i++)
    {
        //write data into flash
        FLASH_ProgramWord(write_addr,*wrtie_data_p);
        write_addr += 4;
        wrtie_data_p++;
    }
    
    FLASH_Lock();

}

void clear_flash_tag()
{
    FLASH_Unlock();
    
    FLASH_ErasePage(PAGE_ADDR(51));
    
    FLASH_Lock();
}

void brightness_init()
{
    if(is_flash_new())
    {
        printf("flash is new, set default values.\r\n");
        set_flash_tag();

        FLASH_Unlock();
        FLASH_ErasePage(PAGE_ADDR(50));

        uint32_t write_addr = PAGE_ADDR(50);

        //set to default
        g_setting[mode_index] = 2;
        g_setting[brightness_index] = 102;    
        g_setting[mode2ratio2d_index] = 100;  //1.00
        g_setting[mode2ratio3d_index] = 15;   //0.15
        g_setting[mode3ratio2d_index] = 0;    //0
        g_setting[mode3ratio3d_index] = 100;  //1.00
        g_setting[ctrlmode2d_index] = I2C_CTRL;

        uint32_t *wrtie_data_p = g_setting;
        for(int i = 0;i < STORE_LEN;i++)
        {
            //write data into flash
            FLASH_ProgramWord(write_addr,*wrtie_data_p);
            write_addr += 4;
            wrtie_data_p++;

        }
        
        set_sys_brightness(g_setting[brightness_index]);
    }
    else
    {
        get_flash_setting();
        if(g_setting[ctrlmode2d_index] == PWM_CTRL)
        {
            //pwm only mode
            uint8_t value = 1;	//0x01 pwm only set brightness,0x5 I2c set brightness
            uint8_t address = 0x01;
            uint8_t read_data = 0;
            ee_WRITE_BYTES(address, &value, 1);
            ee_READ_BYTES(address, &read_data, 1);
            if(value == read_data)
            {
                printf("use pwm to control 8556.\r\n");
				
            }
            else
            {
                printf("set pwm to control 8556 fail.\r\n");
            }
        }
        set_sys_brightness(g_setting[brightness_index]);
    }

}

void init()
{
    int ret = 0;
    USART_Config();
    ADVANCE_TIM_GPIO_Config();
    //ADVANCE_2D_PWM_Config(2048);

    printf("init...\r\n");
    Usart_SendString(DEBUG_USARTx, "pwm configuration is finished.\r\n");
    i2c_GPIO_Config();
    delay(500);

    if(ee_CHECK_DEVICE(WRITE_DIR_8556) == 0)
    {
        printf("8556 has been detected.\r\n");
    }
    else
    {
        printf("8556 has not been detected\r\n");
    }

    delay(500);

    ret = init_chip_8556();
    if(ret != 0)
    {
        printf("[ERROR]8556 init fail.\r\n");
    }

    brightness_init();

}



void set_sys_brightness(uint8_t brightness)
{
    //set 2d brightness,use i2c to set brightness register
    uint8_t blu_brightness_2d = 0;
    if(g_setting[mode_index] == 2)
    {
        blu_brightness_2d = (double)brightness * ((double)g_setting[mode2ratio2d_index] / 100.0);
    }
    else
    {
        blu_brightness_2d = (double)brightness * ((double)g_setting[mode3ratio2d_index] / 100.0);
    }

    //set 3d brightness, use pwm duty to set brightness
    int blu_brightness_3d = 0;
    if(g_setting[mode_index] == 2)
    {
        blu_brightness_3d = (double)brightness * ((double)g_setting[mode2ratio3d_index] / 100.0);
    }
    else
    {
        blu_brightness_3d = (double)brightness * ((double)g_setting[mode3ratio3d_index] / 100.0);
    }

    if(blu_brightness_3d > 255)
    {
        blu_brightness_3d = 255;
    }
    
    printf("system brightness:%d\r\n",brightness);

    printf("brightness_2D:%d brightness_3D:%d\r\n",blu_brightness_2d,blu_brightness_3d);

    int duty_3d = (blu_brightness_3d / 255.0) * DUTY_MAX_3D;
    
    uint8_t ibrightness_read = 0;

    if(g_setting[ctrlmode2d_index] == I2C_CTRL)
    {
        ee_WRITE_BYTES(BRIGHTNESS_CONTROL,&blu_brightness_2d,1);
        
        ee_READ_BYTES(BRIGHTNESS_CONTROL,&ibrightness_read,1);
        
        if(blu_brightness_2d == ibrightness_read)
        {
            usart_send_line("2d i2c set brightness successfully!");
        }
        else
        {
            usart_send_line("2d set i2c set brightness failed!");
        }
        //ADVANCE_2D_PWM_Config(2048);
        ADVANCE_3D_PWM_Config(duty_3d);
        printf("PWM duty_3d:%d/4096\r\n",duty_3d);
    }
    else
    {
        int duty_2d = (blu_brightness_2d / 255.0) * 4096;
        ADVANCE_2D_PWM_Config(duty_2d);
        ADVANCE_3D_PWM_Config(duty_3d);
        printf("duty_2d:%d/4096 duty_3d:%d/4096\r\n",duty_2d,duty_3d);
    }
    
}

void mode_switch()
{
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        return;
    }

    int mode = atoi(g_tokens[1]);
    if(mode != 2 && mode != 3)
    {
        usart_send_line("[ERROR]mode must be 2 or 3!");
        return;
    }

    g_setting[mode_index] = mode;
    save_setting_in_flash();
    set_sys_brightness(g_setting[brightness_index]);
}

void get_flash_setting()
{
    uint32_t *read_p = (uint32_t *)PAGE_ADDR(51);
    //printf("tag:%d\r\n",*read_p);

    read_p = (uint32_t *)PAGE_ADDR(50);
    for(int i = 0; i < STORE_LEN; i++)
    {
        g_setting[i] = read_p[i];
        //printf("g_setting[%d]:%d\r\n",i,g_setting[i]);
    }
}

void save_setting_in_flash()
{
    FLASH_Unlock();

    FLASH_ErasePage(PAGE_ADDR(50));

    uint32_t write_addr = PAGE_ADDR(50);
    uint32_t *wrtie_data_p = g_setting;
    uint32_t *read_p;
    int i = 0;

    for(i = 0;i < STORE_LEN;i++)
    {
        //write data into flash
        FLASH_ProgramWord(write_addr,*wrtie_data_p);
        write_addr += 4;
        wrtie_data_p++;
    }

    read_p = (uint32_t *)PAGE_ADDR(50);
    for(i = 0; i < STORE_LEN; i++)
    {
        //printf("\r\nwrite[%d] = %d, read[%d] = %d",i,g_setting[i],i,read_p[i]);
        if(g_setting[i] != read_p[i])
        {
            break;
        }
    }
    
    if(i == STORE_LEN)
    {
        printf("Write Data to flash success.\r\n");
    }
    
    FLASH_Lock();

}

void mode_set_ratio()
{
    if(cmd_key_num != 4)
    {
        usart_send_line(WRONG_FORMAT);
        return;
    }

    int mode = atoi(g_tokens[1]);
    char *str1,*str2 = NULL;
    double ratio_2d = strtod(g_tokens[2],&str1);
    double ratio_3d = strtod(g_tokens[3],&str2);

    if((mode != 2 && mode != 3) || (*str1 != '\0') || (*str2 != '\0'))
    {
        usart_send_line("mode or ratio illegal");
        return;
    }

    if(ratio_2d > 1.00 || ratio_2d < 0.00 || ratio_3d > 3.00 || ratio_3d < 0.00)
    {
        usart_send_line("ratio error,ratio_2d:0.00~1.00 ,ratio_3d:0.00~3.00");
        return;
    }

    if(mode == 2)
    {
        g_setting[mode2ratio2d_index] = (int)(ratio_2d * 100);
        g_setting[mode2ratio3d_index] = (int)(ratio_3d * 100);
    }
    else
    {
        g_setting[mode3ratio2d_index] = (int)(ratio_2d * 100);
        g_setting[mode3ratio3d_index] = (int)(ratio_3d * 100);
    }

    //save in stm32's flash
    save_setting_in_flash();
    set_sys_brightness(g_setting[brightness_index]);
    
}

void display_boardinfo()
{
    uint32_t read_addr = PAGE_ADDR(51) + 4;
    char *p = (char *)read_addr;
    char info[128] = "";
    for(int i = 0; i < 128; i++)
    {
        info[i] = *p;
        p++;
    }
    
    printf("BOARD INFO:\r\n%s\r\n",info);
}

void setting_get()
{
    if(cmd_key_num != 1)
    {
        usart_send_line(WRONG_FORMAT);
        return;
    }

    get_flash_setting();
    int brightness = g_setting[brightness_index];
    int mode = g_setting[mode_index];
    double mode2_ratio_2d = g_setting[mode2ratio2d_index] / 100.0;
    double mode2_ratio_3d = g_setting[mode2ratio3d_index] / 100.0;
    double mode3_ratio_2d = g_setting[mode3ratio2d_index] / 100.0;
    double mode3_ratio_3d = g_setting[mode3ratio3d_index] / 100.0;
    
    double current_2d = (mode == 2) ? ((double)brightness * mode2_ratio_2d /255.0 * 25.00) : ((double)brightness * mode3_ratio_2d /255.0 * 25.00);
    double current_3d = (mode == 2) ? ((double)brightness * mode2_ratio_3d /255.0 * 14.00) : ((double)brightness * mode3_ratio_3d /255.0 * 14.00);
    printf("Display current system configurations:\r\n");
    if(g_setting[ctrlmode2d_index] == I2C_CTRL)
    {
        printf("brightness: %d\r\n2d control mode: i2c\r\n",brightness);
    }
    else
    {
        printf("brightness: %d\r\n2d control mode: pwm\r\n",brightness);
    }

    printf("+------+----------+----------+------------+------------+\r\n");
    printf("| mode | 2D_ratio | 3D_ratio | 2D_current | 3D_current |\r\n");
    if(mode == 2)
    {
        printf("+------+----------+----------+------------+------------+\r\n");
        printf("|%-6d|%-10.2f|%-10.2f|%-12.2f|%-12.2f|\r\n",2,mode2_ratio_2d,mode2_ratio_3d,current_2d,current_3d);
    }
    else
    {
        printf("+------+----------+----------+------------+------------+\r\n");
        printf("|%-6d|%-10.2f|%-10.2f|%-12.2f|%-12.2f|\r\n",3,mode3_ratio_2d,mode3_ratio_3d,current_2d,current_3d);
    }

    printf("+------+----------+----------+------------+------------+\r\n\r\n");
}

void set_2d_current()
{
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        return;
    }  

    char *str = NULL;
    double current = strtod(g_tokens[1],&str);

    if((*str != '\0'))
    {
        usart_send_line("current illegal");
        return;
    }

    if(current > 25.00 || current < 0.00)
    {
        usart_send_line("current error,current:0.00~25.00");
        return;
    }
    
    int brightness = current / 25.00 * 255;
    uint8_t ibrightness_read = 0;

    if(g_setting[ctrlmode2d_index] == I2C_CTRL)
    {
        uint8_t brightness_set = (uint8_t)brightness;
        ee_WRITE_BYTES(BRIGHTNESS_CONTROL,&brightness_set,1);
    
        ee_READ_BYTES(BRIGHTNESS_CONTROL,&ibrightness_read,1);
    
        if(brightness == ibrightness_read)
        {
            printf("2d set successfully! This setting is temporary. Now temp brightness is %d\r\n",ibrightness_read);
        }
        else
        {
            usart_send_line("2d set failed!");
        }
    }
    else
    {
        int duty_2d = (current / 25.00) * 4096; 
        ADVANCE_2D_PWM_Config(duty_2d); 
        printf("duty_2d :%d/4096\r\n",duty_2d);
    }
    
}

void set_3d_current()
{
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        return;
    }  

    char *str = NULL;
    double current = strtod(g_tokens[1],&str);

    if((*str != '\0'))
    {
        usart_send_line("current illegal\r\n");
		return;
    }

    if(current > 14.00 || current < 0.00)
    {
        usart_send_line("current error,current:0.00~14.00\r\n");
		return;
    }

    int duty_3d = (current / 14.00) * DUTY_MAX_3D;
    ADVANCE_3D_PWM_Config(duty_3d); 
    printf("duty_3d :%d/4096\r\n",duty_3d);

}

void set_2dctrmode()
{
    if(cmd_key_num != 2)
    {
        usart_send_line(WRONG_FORMAT);
        return;
    }

    int mode = atoi(g_tokens[1]);
    if(mode != 0 && mode != 1)
    {
        printf("[ERROR]mode should be 0(i2c) or 1(pwm).\r\n");
        return;
    }

    uint8_t read_data = 0;
    uint8_t address = 0x01;
    uint8_t value = 0;

    g_setting[ctrlmode2d_index] = mode;
    save_setting_in_flash();
    if(mode == I2C_CTRL)
    {
        //i2c only mode
        value = 5;
        ee_WRITE_BYTES(address, &value, 1);
        ee_READ_BYTES(address, &read_data, 1);
        if(value == read_data)
        {
            printf("use i2c to control 8556.\r\n");
        }
        else
        {
            printf("set i2c to control 8556 fail.\r\n");
        }
    }
    else
    {
        //pwm only mode
        value = 1;
        ee_WRITE_BYTES(address, &value, 1);
        ee_READ_BYTES(address, &read_data, 1);
        if(value == read_data)
        {
        	int blu_brightness_2d = 0;
			if(g_setting[mode_index] == 2)
			{
				blu_brightness_2d = (double)g_setting[brightness_index] * ((double)g_setting[mode2ratio2d_index] / 100.0);
			}
			else
			{
				blu_brightness_2d = (double)g_setting[brightness_index] * ((double)g_setting[mode3ratio2d_index] / 100.0);
			}
            printf("use pwm to control 8556.\r\n");
			int duty_2d = (blu_brightness_2d / 255.0) * 4096;
			ADVANCE_2D_PWM_Config(duty_2d);
        }
        else
        {
            printf("set pwm to control 8556 fail.\r\n");
        }
    }
    
}

void set_pwm()
{
    if(cmd_key_num != 3)
    {
        usart_send_line(WRONG_FORMAT);
        return;
    }  

    int mode = atoi(g_tokens[1]);
    int duty = atoi(g_tokens[2]);

    if((mode != 2 && mode != 3))
    {
        usart_send_line("mode illegal");
        return;
    }

    if(duty < 0.00 || duty > 4096)
    {
        usart_send_line("duty error,duty can only be 0~4096");
        return;
    }

    if(mode == 2)
    {
        ADVANCE_2D_PWM_Config(duty);  
    }
    else
    {
        ADVANCE_3D_PWM_Config(duty);  
    }
}

void gpio_read()
{
    //TODO
}

void gpio_readpin()
{
    //TODO
}

void gpio_setpin()
{
    //TODO
}

void gpio_clrpin()
{
    //TODO
}

void factoryreset()
{
    //set flash memory
    clear_flash_tag();
    init();
    printf("Reset to Factory Defaults.\r\n");
}

void clear_cmd_info(void)
{
    cmd_id = CMD_NULL;
    cmd_key_num = 0;
    memset(g_usart_buf,0,COMMAND_MAX_LENGTH);
}

void command_parse() {

    int i = 0;
    printf("\r\nCMD:%s\r\n",g_usart_buf);
    memset(g_tokens,0,5 * sizeof(char *));
    
    for (char *p = strtok(g_usart_buf," "); p != NULL; p = strtok(NULL, " "))
    {
        g_tokens[i] = p;
        //printf("CMD KEY:%s\t",g_tokens[i]);
        i++;
        cmd_key_num = i;
    }
    //printf("\n");
    if(!strncmp(g_tokens[0],"HELP",strlen("HELP")))
    {
        cmd_id = HELP;
    }
    else if(!strncmp(g_tokens[0],"INFO",strlen("INFO")))
    {
        cmd_id = INFO;
    }
    else if(!strncmp(g_tokens[0],"DUMP",strlen("DUMP")))
    {
        cmd_id = DUMP;
    }
    else if(!strncmp(g_tokens[0],"I2CREAD",strlen("I2CREAD")))
    {
        cmd_id = I2CREAD;
    }
    else if(!strncmp(g_tokens[0],"I2CWRITE",strlen("I2CWRITE")))
    {
        cmd_id = I2CWRITE;
    }
    else if(!strncmp(g_tokens[0],"BLSETBRIGHTNESS",strlen("BLSETBRIGHTNESS")))
    {
        cmd_id = BLSETBRIGHTNESS;
    }
    else if(!strncmp(g_tokens[0],"BLSETRATIOS",strlen("BLSETRATIOS")))
    {
        cmd_id = BLSETRATIOS;
    }
    else if(!strncmp(g_tokens[0],"BLSWITCH",strlen("BLSWITCH")))
    {
        cmd_id = BLSWITCH;
    }
    else if(!strncmp(g_tokens[0],"SET2DCTRLMODE",strlen("SET2DCTRLMODE")))
    {
        cmd_id = SET2DCTRLMODE;
    }
    else if(!strncmp(g_tokens[0],"BLSETPWM",strlen("BLSETPWM")))
    {
        cmd_id = BLSETPWM;
    }
    else if(!strncmp(g_tokens[0],"BLSETTINGSGET",strlen("BLSETTINGSGET")))
    {
        cmd_id = BLSETTINGSGET;
    }
    else if(!strncmp(g_tokens[0],"BLSET2DCURRENT",strlen("BLSET2DCURRENT")))
    {
        cmd_id = BLSET2DCURRENT;
    }
    else if(!strncmp(g_tokens[0],"BLSET3DCURRENT",strlen("BLSET3DCURRENT")))
    {
        cmd_id = BLSET3DCURRENT;
    }
    else if(!strncmp(g_tokens[0],"POKE32",strlen("POKE32")))
    {
        cmd_id = POKE32;
    }
    else if(!strncmp(g_tokens[0],"POKE16",strlen("POKE16")))
    {
        cmd_id = POKE16;
    }
    else if(!strncmp(g_tokens[0],"POKE8",strlen("POKE8")))
    {
        cmd_id = POKE8;
    }
    else if(!strncmp(g_tokens[0],"PEEK32",strlen("PEEK32")))
    {
        cmd_id = PEEK32;
    }
    else if(!strncmp(g_tokens[0],"PEEK16",strlen("PEEK16")))
    {
        cmd_id = PEEK16;
    }
    else if(!strncmp(g_tokens[0],"PEEK8",strlen("PEEK8")))
    {
        cmd_id = PEEK8;
    }
    else if(!strncmp(g_tokens[0],"GPIOREAD",strlen("GPIOREAD")))
    {
        cmd_id = GPIOREAD;
    }
    else if(!strncmp(g_tokens[0],"GPIOREADPIN",strlen("GPIOREADPIN")))
    {
        cmd_id = GPIOREADPIN;
    }
    else if(!strncmp(g_tokens[0],"GPIOSETPIN",strlen("GPIOSETPIN")))
    {
        cmd_id = GPIOSETPIN;
    }
    else if(!strncmp(g_tokens[0],"GPIOCLRPIN",strlen("GPIOCLRPIN")))
    {
        cmd_id = GPIOCLRPIN;
    }
    else if(!strncmp(g_tokens[0],"BLFACTORYRESET",strlen("BLFACTORYRESET")))
    {
        cmd_id = BLFACTORYRESET;
    }
    else
    {
        cmd_id = UNKNOWCMD;
    }
}


void handle_command()
{
    switch(cmd_id){
        case HELP:
        {
            printf("\r\nShow All Commands:\r\n%s\r\n",cmd_help_str);
            clear_cmd_info();
            break;
        }      
        case DUMP:
        {                
            dump_chip_8556();
            clear_cmd_info();
            break;
        }
        case INFO:
        {                
            display_boardinfo();
            clear_cmd_info();
            break;
        }
        case I2CREAD:
        {
            int ret = i2c_read_8556();
            if (ret != 0)
                printf("Command I2CREAD fail.\r\n");
            clear_cmd_info();
            break;
        }
        case I2CWRITE:
        {
            int ret = i2c_write_8556();
            if (ret != 0)
                printf("Command I2CWRITE fail.\r\n");
            clear_cmd_info();
            break;
        }
        case BLSWITCH:
        {
            mode_switch();
            clear_cmd_info();
            break;
        }
        case BLSETBRIGHTNESS:
        {
            int brightness = atoi(g_tokens[1]);
            if(brightness > 255 || brightness < 0)
            {
                printf("[ERROR]brightness value should be 0~255.\r\n");
                break;
            }
            set_sys_brightness(brightness);
            g_setting[brightness_index] = brightness;
            save_setting_in_flash();

            clear_cmd_info();
            break;
        }
        case BLSETRATIOS:
        {
            mode_set_ratio();
            clear_cmd_info();
            break;
        }
        case BLSETTINGSGET:
        {
            setting_get();
            clear_cmd_info();      
            break;
        }
        case BLSET2DCURRENT:
        {
            set_2d_current();
            clear_cmd_info();       
            break;  
        }
        case BLSET3DCURRENT:
        {
            set_3d_current();
            clear_cmd_info();       
            break;  
        }
        case BLSETPWM:
        {
            set_pwm();
            clear_cmd_info();       
            break; 
        }
        case SET2DCTRLMODE:
        {
            set_2dctrmode();
            clear_cmd_info();       
            break; 
        }
        case POKE32:
        {
            char *str1 = NULL;
            char *str2 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            int data_input = strtol(g_tokens[2],&str2,16);

            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 32 bit HEX num!");
                break;
            }

            if((*str2) != '\0')
            {
                usart_send_line("[ERROR]data can only be a 32 bit HEX num!");
                break;
            }

            uint32_t *p = (uint32_t *)address_input;
            *p = data_input;
            clear_cmd_info();
            break; 
        }
        case POKE16:
        {
            char *str1 = NULL;
            char *str2 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            int data_input = strtol(g_tokens[2],&str2,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 16 bit HEX num!");
                break;
            }

            if((*str2) != '\0' || data_input < 0 || data_input > 65535)
            {
                usart_send_line("[ERROR]data can only be a 16 bit HEX num!");
                break;
            }

            uint16_t *p = (uint16_t *)address_input;
            *p = data_input;

            clear_cmd_info();     
            break; 
        }
        case POKE8:
        {
            char *str1 = NULL;
            char *str2 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            int data_input = strtol(g_tokens[2],&str2,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 8 bit HEX num!");
                break;
            }

            if((*str2) != '\0' || data_input < 0 || data_input > 255)
            {
                usart_send_line("[ERROR]data can only be a 8 bit HEX num!");
                break;
            }

            uint8_t *p = (uint8_t *)address_input;
            *p = data_input;

            clear_cmd_info();        
            break; 
        }
        case PEEK32:
        {
            char *str1 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 32 bit HEX num!");
                break;
            }

            uint32_t *p = (uint32_t *)address_input;
            printf("address:0x%x, data:0x%x\r\n",address_input,*p);

            clear_cmd_info();
            break; 
        }
        case PEEK16:
        {
            char *str1 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 32 bit HEX num!");
                break;
            }

            uint16_t *p = (uint16_t *)address_input;
            printf("address:0x%x, data:0x%x\r\n",address_input,*p);

            clear_cmd_info();       
            break; 
        }
        case PEEK8:
        {
            char *str1 = NULL;
            int address_input = strtol(g_tokens[1],&str1,16);
            
            if((*str1) != '\0')
            {
                usart_send_line("[ERROR]address can only be a 32 bit HEX num!");
                break;
            }

            uint8_t *p = (uint8_t *)address_input;
            printf("address:0x%x, data:0x%x\r\n",address_input,*p);

            clear_cmd_info();
            break; 
        }
        case GPIOREAD:
        {
            gpio_read();
            clear_cmd_info();
            break;
        }
        case GPIOREADPIN:
        {
            gpio_readpin();
            clear_cmd_info();
            break;
        }
        case GPIOSETPIN:
        {
            gpio_setpin();
            clear_cmd_info();
            break;
        }
        case GPIOCLRPIN:
        {
            gpio_clrpin();
            clear_cmd_info();
            break; 
        }
        case BLFACTORYRESET:
        {
            factoryreset();
            clear_cmd_info();
            break; 
        }
        case CMD_NULL:
            break;
        default:
            break;
    }
}   

int main(void)
{
    init();

    while(1)
    {
        if(cmd_received)
        {
            command_parse();
            handle_command();
            cmd_received = 0;
        }
    }
}



