#include "SD.h"
#include "SPI.h"
#include "picture.h"

#define SPI_MOSI        23
#define SPI_MISO        2
#define SPI_CLK         18

#define EINK_SS        5
#define EINK_BUSY      4
#define EINK_RESET     12
#define EINK_DC        19

#define SDCARD_SS       13

#define BUTTON_1        37
#define BUTTON_2        38
#define BUTTON_3        39

#define SPEAKER_OUT     25
#define CHANNEL_0       0

#define POS 1
#define NEG 2

#define MAX_LINE_BYTES          16// =128/8
#define MAX_COLUMN_BYTES        296
#define ALLSCREEN_GRAGHBYTES    4736


void Epaper_READBUSY(void)
{
    while (1) {
        //=1 BUSY
        if (digitalRead(EINK_BUSY) == 0) break;;
    }
}

void Epaper_Write_Command(unsigned char cmd)
{
    digitalWrite(EINK_SS, HIGH);
    digitalWrite(EINK_SS, LOW);
    digitalWrite(EINK_DC, LOW); // D/C#   0:command  1:data
    SPI.transfer(cmd);
    digitalWrite(EINK_SS, HIGH);
}

void Epaper_Write_Data(unsigned char data)
{
    digitalWrite(EINK_SS, HIGH);
    digitalWrite(EINK_SS, LOW);
    digitalWrite(EINK_DC, HIGH); // D/C#   0:command  1:data
    SPI.transfer(data);
    digitalWrite(EINK_SS, HIGH);
}

/////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

void EPD_HW_Init(void)
{
    digitalWrite(EINK_RESET, LOW); // Module reset
    delay(100);//At least 10ms delay
    digitalWrite(EINK_RESET, HIGH);
    delay(100); //At least 10ms delay

    Epaper_READBUSY();
    Epaper_Write_Command(0x12); // soft reset
    Epaper_READBUSY();

    Epaper_Write_Command(0x01); //Driver output control
    Epaper_Write_Data(0x27);
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x00);

    Epaper_Write_Command(0x11); //data entry mode
    Epaper_Write_Data(0x01);        //Y decrement, X increment

    Epaper_Write_Command(0x44); //set Ram-X address start/end position
    Epaper_Write_Data(0x01);        //0x00-->0
    Epaper_Write_Data(0x10);    //0x0F-->(15+1)*8=128

    Epaper_Write_Command(0x45); //set Ram-Y address start/end position
    Epaper_Write_Data(0x27);    //0x127-->(295+1)=296
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x00);

    Epaper_Write_Command(0x3C); //BorderWavefrom
    Epaper_Write_Data(0x01);

    Epaper_Write_Command(0x18); //Temperature Sensor Selection
    Epaper_Write_Data(0x80);      //Internal temperature sensor

    Epaper_Write_Command(0x4E);   // set RAM x address count
    Epaper_Write_Data(0x01);
    Epaper_Write_Command(0x4F);   // set RAM y address count
    Epaper_Write_Data(0x27);
    Epaper_Write_Data(0x01);
    Epaper_READBUSY();

}
/////////////////////////////////////////////////////////////////////////////////////////
void EPD_Update(void)
{
    Epaper_Write_Command(0x22);
    Epaper_Write_Data(0xF7);
    Epaper_Write_Command(0x20);
    Epaper_READBUSY();

}

//////////////////////////////All screen update////////////////////////////////////////////
void EPD_WhiteScreen_ALL(const unsigned char *datas)
{
    unsigned int i;
    Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(*datas);
        datas++;
    }
    EPD_Update();
}



void EPD_DeepSleep(void)
{
    Epaper_Write_Command(0x10); //enter deep sleep
    Epaper_Write_Data(0x01);
    delay(100);
}
///////////////////////////Part update//////////////////////////////////////////////
//The x axis is reduced by one byte, and the y axis is reduced by one pixel.
void EPD_SetRAMValue_BaseMap( const unsigned char *datas)
{
    unsigned int i;
    const unsigned char  *datas_flag;
    datas_flag = datas;
    Epaper_Write_Command(0x24);   //Write Black and White image to RAM
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(*datas);
        datas++;
    }
    datas = datas_flag;
    Epaper_Write_Command(0x26);   //Write Black and White image to RAM
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(*datas);
        datas++;
    }
    EPD_Update();
}


void EPD_Dis_Part(unsigned int ystart, unsigned int xstart, const unsigned char *datas, unsigned int PART_COLUMN, unsigned int PART_LINE, unsigned char mode)
{
    unsigned int i;
    int xend, ystart_H, ystart_L, yend, yend_H, yend_L;

    xstart = xstart / 8;
    xend = xstart + PART_LINE / 8 - 1;

    ystart = 295 - ystart;
    ystart_H = ystart / 256;
    ystart_L = ystart % 256;

    yend = ystart - PART_COLUMN + 1;
    yend_H = yend / 256;
    yend_L = yend % 256;

    Epaper_Write_Command(0x44);       // set RAM x address start/end
    Epaper_Write_Data(xstart + 1);      // RAM x address start;
    Epaper_Write_Data(xend + 1);        // RAM x address end
    Epaper_Write_Command(0x45);       // set RAM y address start/end
    Epaper_Write_Data(ystart_L);        // RAM y address start Low
    Epaper_Write_Data(ystart_H);        // RAM y address start High
    Epaper_Write_Data(yend_L);          // RAM y address end Low
    Epaper_Write_Data(yend_H);          // RAM y address end High


    Epaper_Write_Command(0x4E);         // set RAM x address count
    Epaper_Write_Data(xstart + 1);
    Epaper_Write_Command(0x4F);         // set RAM y address count
    Epaper_Write_Data(ystart_L);
    Epaper_Write_Data(ystart_H);


    Epaper_Write_Command(0x24);   //Write Black and White image to RAM

    for (i = 0; i < PART_COLUMN * PART_LINE / 8; i++) {
        if (mode == POS) {
            Epaper_Write_Data(*datas);
            datas++;
        }

        if (mode == NEG) {
            Epaper_Write_Data(~*datas);
            datas++;
        }
    }
}

void EPD_Part_Update(void)
{
    Epaper_Write_Command(0x22);
    Epaper_Write_Data(0xFF);
    Epaper_Write_Command(0x20);
    Epaper_READBUSY();
}

/////////////////////////////////Single display////////////////////////////////////////////////

void EPD_WhiteScreen_White(void)

{
    unsigned int k;
    Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)
    for (k = 0; k < ALLSCREEN_GRAGHBYTES; k++) {
        Epaper_Write_Data(0xff);
    }

    Epaper_Write_Command(0x26);   //write RAM for black(0)/white (1)
    for (k = 0; k < ALLSCREEN_GRAGHBYTES; k++) {
        Epaper_Write_Data(0xff);
    }
    EPD_Update();
}


//////////////////////////////////////////////////////////////////////////////////////


void setup()
{
    Serial.begin(115200);

    pinMode(BUTTON_1, INPUT);
    pinMode(BUTTON_2, INPUT);
    pinMode(BUTTON_3, INPUT);

    pinMode(EINK_BUSY, INPUT);
    pinMode(EINK_RESET, OUTPUT);
    pinMode(EINK_DC, OUTPUT);
    pinMode(EINK_SS, OUTPUT);

    ledcSetup(CHANNEL_0, 1000, 8);
    ledcAttachPin(SPEAKER_OUT, CHANNEL_0);
    int i = 3;
    while (i--) {
        ledcWriteTone(CHANNEL_0, 1000);
        delay(200);
        ledcWriteTone(CHANNEL_0, 0);
    }

    SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI);

    if (SD.begin(SDCARD_SS)) {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.println("SDCard:" + String(cardSize) + "MB");
    } else {
        Serial.println("SDCard  None");
    }

    EPD_HW_Init();
    EPD_WhiteScreen_ALL(logo1);
    EPD_DeepSleep();
    delay(3000);

    EPD_HW_Init();
    EPD_WhiteScreen_ALL(logo2);
    EPD_DeepSleep();
    delay(3000);

    //Full screen refresh
    EPD_HW_Init();                                                  //Electronic paper initialization
    EPD_WhiteScreen_ALL(gImage_1);                  //Refresh the picture in full screen
    EPD_DeepSleep();
    delay(3000);

    EPD_HW_Init();                                                  //Electronic paper initialization
    EPD_SetRAMValue_BaseMap(gImage_base);   //Partial refresh background（注意：此处刷背景图片的函数和全刷图片的函数不一样）
    delay(500);


    EPD_HW_Init(); //Electronic paper initialization
    EPD_Dis_Part(29, 32, BLACK, 33, 64, POS);                    //y,x,-1,Resolution 33*64                         数字-1 OFF
    EPD_Dis_Part(25, 104, gImage_Celsius, 47, 16, NEG); //y,x,Celsius,Resolution 47*16              摄氏度负显
    EPD_Dis_Part(75, 32, gImage_num5, 33, 64, POS);     //y,x,num5,Resolution 33*64                     数字5
    EPD_Dis_Part(120, 32, gImage_num1, 33, 64, POS);    //y,x,num1,Resolution 33*64                     数字1
    EPD_Dis_Part(160, 88, BLACK, 8, 8, NEG);             //y,x,dot,Resolution 8*8                            小数点（BLACK，POS不显；BLACK，NEG显示）
    EPD_Dis_Part(175, 32, gImage_num9, 33, 64, POS);    //y,x,num9,Resolution 33*64                     数字9
    EPD_Dis_Part(221, 32, gImage_degree, 10, 16, POS); //y,x,degree,Resolution 10*16                 °
    EPD_Dis_Part(233, 32, gImage_C, 33, 64, POS);           //y,x,C,Resolution 33*64                            C
    EPD_Dis_Part(243, 16, BLACK, 4, 8, POS);             //y,x,battery_1,Resolution 4*8              电量最高格（0x00显示，0xFF不显）;
    EPD_Part_Update();

    delay(1500);

    EPD_Dis_Part(25, 104, gImage_Celsius, 47, 16, POS); //y,x,Celsius,Resolution 47*16              摄氏度正显
    EPD_Dis_Part(92, 104, gImage_Fahrenheit, 47, 16, NEG); //y,x,Fahrenheit,Resolution 47*16         华氏度负显
    EPD_Dis_Part(29, 32, gImage_num1, 33, 64, POS);     //y,x,num1,Resolution 33*64                     数字1
    EPD_Dis_Part(75, 32, gImage_num2, 33, 64, POS);     //y,x,num2,Resolution 33*64                     数字2
    EPD_Dis_Part(120, 32, gImage_num5, 33, 64, POS);    //y,x,num5,Resolution 33*64                     数字5
    EPD_Dis_Part(160, 88, BLACK, 8, 8, NEG);             //y,x,dot,Resolution 8*8                            小数点（BLACK，POS不显；BLACK，NEG显示）
    EPD_Dis_Part(175, 32, gImage_num4, 33, 64, POS);    //y,x,num4,Resolution 33*64                     数字4
    EPD_Dis_Part(221, 32, gImage_degree, 10, 16, POS); //y,x,degree,Resolution 10*16                 °
    EPD_Dis_Part(233, 32, gImage_F, 33, 64, POS);           //y,x,F,Resolution 33*64                            F
    EPD_Dis_Part(243, 16, BLACK, 4, 8, POS);             //y,x,battery_1,Resolution 4*8              电量最高格（0x00显示，0xFF不显）;

    EPD_Part_Update();
    delay(1500);

    EPD_Dis_Part(221, 32, BLACK, 10, 16, POS);                   //y,x,degree,Resolution 10*16                   ° OFF
    EPD_Dis_Part(233, 32, BLACK, 33, 64, POS);               //y,x,F,Resolution 33*64                            F   OFF
    EPD_Dis_Part(29, 32, BLACK, 33, 64, POS);              //y,x,num1,Resolution 33*64                       数字-1 OFF

    EPD_Dis_Part(25, 104, gImage_Celsius, 47, 16, POS); //y,x,Celsius,Resolution 47*16              摄氏度正显
    EPD_Dis_Part(92, 104, gImage_Fahrenheit, 47, 16, POS); //y,x,Fahrenheit,Resolution 47*16         华氏度正显
    EPD_Dis_Part(159, 104, gImage_humidity, 47, 16, NEG); //y,x,humidity,Resolution 47*16             湿度负显
    EPD_Dis_Part(75, 32, gImage_num3, 33, 64, POS);         //y,x,num3,Resolution 33*64                     数字3
    EPD_Dis_Part(120, 32, gImage_num0, 33, 64, POS);      //y,x,num0,Resolution 33*64                       数字0
    EPD_Dis_Part(160, 88, BLACK, 8, 8, NEG);             //y,x,dot,Resolution 8*8                            小数点（BLACK，POS不显；BLACK，NEG显示）
    EPD_Dis_Part(175, 32, gImage_num6, 33, 64, POS);        //y,x,num6,Resolution 33*64                     数字6
    EPD_Dis_Part(223, 32, gImage_Percent, 44, 64, POS); //x,y,%,Resolution 44*64                            百分号
    EPD_Dis_Part(243, 16, BLACK, 4, 8, POS);             //y,x,battery_1,Resolution 4*8              电量最高格（0x00显示，0xFF不显）;

    EPD_Part_Update();
    delay(1500);
    EPD_Dis_Part(221, 32, BLACK, 10, 16, POS);                   //y,x,degree,Resolution 10*16                   ° OFF
    EPD_Dis_Part(233, 32, BLACK, 33, 64, POS);               //y,x,F,Resolution 33*64                            F   OFF
    EPD_Dis_Part(29, 32, BLACK, 33, 64, POS);              //y,x,num1,Resolution 33*64                       数字-1 OFF

    EPD_Dis_Part(25, 104, gImage_Celsius, 47, 16, POS); //y,x,Celsius,Resolution 47*16              摄氏度正显
    EPD_Dis_Part(92, 104, gImage_Fahrenheit, 47, 16, POS); //y,x,Fahrenheit,Resolution 47*16         华氏度正显
    EPD_Dis_Part(159, 104, gImage_humidity, 47, 16, POS); //y,x,humidity,Resolution 47*16             湿度正显
    EPD_Dis_Part(226, 104, gImage_Power, 47, 16, NEG);  //y,x,Power,Resolution 47*16                  电量负显
    EPD_Dis_Part(75, 32, gImage_num7, 33, 64, POS);         //y,x,num7,Resolution 33*64                     数字7
    EPD_Dis_Part(120, 32, gImage_num2, 33, 64, POS);      //y,x,num2,Resolution 33*64                       数字2
    EPD_Dis_Part(160, 88, BLACK, 8, 8, NEG);             //y,x,dot,Resolution 8*8                            小数点（BLACK，POS不显；BLACK，NEG显示）
    EPD_Dis_Part(175, 32, gImage_num8, 33, 64, POS);        //y,x,num8,Resolution 33*64                     数字8
    EPD_Dis_Part(223, 32, gImage_Percent, 44, 64, POS); //x,y,%,Resolution 44*64                            百分号
    EPD_Dis_Part(243, 16, BLACK, 4, 8, POS);             //y,x,battery_1,Resolution 4*8              电量最高格（BLACK，POS不显；BLACK，NEG显示）;

    EPD_Part_Update();
    EPD_DeepSleep();                                                                //Enter deep sleep
    delay(1500);


///////////////////////////////////////////第六部分
//////////////////////Partial screen refresh/////////////////////////////////////

    EPD_HW_Init();                                                                      //Electronic paper initialization
    EPD_WhiteScreen_White();                                                //Show all white
    EPD_DeepSleep();                                                                //Enter deep sleep

//用局刷方式连续刷新多个整屏图片的时候，只能有第一个初始化，中间不能进入休眠，全部刷新完毕后才能进入休眠。

    EPD_HW_Init(); //Electronic paper initialization
    EPD_Dis_Part(0, 0, gImage_1, 296, 128, POS);              //y,x,gImage_1,Resolution 296*128
    EPD_Part_Update();
    delay(1500);

    EPD_Dis_Part(0, 0, gImage_base, 296, 128, POS);   //y,x,gImage_base,Resolution 296*128
    EPD_Part_Update();
    EPD_DeepSleep();                                                                //Enter deep sleep
    delay(1500);
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
    //Clear screen
    EPD_HW_Init();                                                                      //Electronic paper initialization
    EPD_WhiteScreen_White();                                                //Show all white
    EPD_DeepSleep();
}

void loop()
{
    if (digitalRead(BUTTON_1) == LOW) {
        Serial.printf("%d Pressed\n", BUTTON_1);
    } else if (digitalRead(BUTTON_2) == LOW) {
        Serial.printf("%d Pressed\n", BUTTON_2);
    } else if (digitalRead(BUTTON_3) == LOW) {
        Serial.printf("%d Pressed\n", BUTTON_3);
        esp_sleep_enable_ext0_wakeup((gpio_num_t )BUTTON_3, LOW);
        esp_deep_sleep_start();
    }
}
