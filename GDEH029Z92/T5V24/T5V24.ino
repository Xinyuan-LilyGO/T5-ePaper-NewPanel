// example code from ： http://www.e-paper-display.cn/products_detail/productId=515.html , Just to test the display
// example code from ： http://www.e-paper-display.cn/products_detail/productId=515.html , Just to test the display
// example code from ： http://www.e-paper-display.cn/products_detail/productId=515.html , Just to test the display


// include library, include base class, make path known
#include "SD.h"
#include "SPI.h"
#include <Button2.h>
#include "picture.h"

#define EINK_BUSY 4
#define EINK_RESET 16
#define EINK_DC 17
#define EINK_SS 5

#define SPI_MOSI 23
#define SPI_MISO -1 //elink no use
#define SPI_CLK 18

#define SDCARD_SS 13
#define SDCARD_MOSI 15
#define SDCARD_MISO 2
#define SDCARD_CLK 14

#define BUTTON_1 37
#define BUTTON_2 38
#define BUTTON_3 39

#define SPEAKER_OUT 25
#define AMP_POWER_CTRL 19
#define CHANNEL_0 0
#define BUTTONS_MAP \
    {               \
        37, 38, 39  \
    }

Button2 *pBtns = nullptr;

uint8_t g_btns[] = BUTTONS_MAP;

SPIClass SDSPI(VSPI);

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

void EPD_HW_Init(void)
{
    digitalWrite(EINK_RESET, LOW); // Module reset
    delay(100);//At least 10ms delay
    digitalWrite(EINK_RESET, HIGH);
    delay(100); //At least 10ms delay

    Epaper_READBUSY();
    Epaper_Write_Command(0x12);  //SWRESET
    Epaper_READBUSY();

    Epaper_Write_Command(0x01); //Driver output control
    Epaper_Write_Data(0x27);
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x00);

    Epaper_Write_Command(0x11); //data entry mode
    Epaper_Write_Data(0x01);

    Epaper_Write_Command(0x44); //set Ram-X address start/end position
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x0F);    //0x0F-->(15+1)*8=128

    Epaper_Write_Command(0x45); //set Ram-Y address start/end position
    Epaper_Write_Data(0x27);   //0x0127-->(295+1)=296
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x00);

    Epaper_Write_Command(0x3C); //BorderWavefrom
    Epaper_Write_Data(0x05);

    Epaper_Write_Command(0x18); //Read built-in temperature sensor
    Epaper_Write_Data(0x80);

    Epaper_Write_Command(0x21); //  Display update control
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x80);

    Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
    Epaper_Write_Data(0x00);
    Epaper_Write_Command(0x4F);   // set RAM y address count to 0X199;
    Epaper_Write_Data(0x27);
    Epaper_Write_Data(0x01);
    Epaper_READBUSY();

}
/////////////////////////////////////////////////////////////////////////////////////////
void EPD_Update(void)
{
    Epaper_Write_Command(0x22); //Display Update Control
    Epaper_Write_Data(0xF7);
    Epaper_Write_Command(0x20);  //Activate Display Update Sequence
    Epaper_READBUSY();

}


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
//////////////////////////////All screen update////////////////////////////////////////////
void EPD_WhiteScreen_ALL(const unsigned char *BW_datas, const unsigned char *R_datas)
{
    unsigned int i;
    Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(*BW_datas);
        BW_datas++;
    }
    Epaper_Write_Command(0x26);   //write RAM for black(0)/white (1)
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(~(*R_datas));
        R_datas++;
    }
    EPD_Update();
}

void EPD_WhiteScreen_ALL_Clean(void)
{
    unsigned int i;
    Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(0xff);
    }
    Epaper_Write_Command(0x26);   //write RAM for black(0)/white (1)
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(0x00);
    }
    EPD_Update();
}
void EPD_DeepSleep(void)
{
    Epaper_Write_Command(0x10); //enter deep sleep
    Epaper_Write_Data(0x01);
    delay(100);
}



void button_callback(Button2 &b)
{
    for (int i = 0; i < sizeof(g_btns) / sizeof(g_btns[0]); ++i) {
        if (pBtns[i] == b) {
            Serial.printf("Button: %u Press\n", pBtns[i].getAttachPin());
            ledcWriteTone(CHANNEL_0, 1000);
            delay(200);
            ledcWriteTone(CHANNEL_0, 0);
        }
    }
}

void button_init()
{
    uint8_t args = sizeof(g_btns) / sizeof(g_btns[0]);
    pBtns = new Button2[args];
    for (int i = 0; i < args; ++i) {
        pBtns[i] = Button2(g_btns[i]);
        pBtns[i].setPressedHandler(button_callback);
    }
}


void setup()
{
    Serial.begin(115200);
    Serial.println();
    Serial.println("setup");
    SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI);

    pinMode(EINK_BUSY, INPUT);
    pinMode(EINK_RESET, OUTPUT);
    pinMode(EINK_DC, OUTPUT);
    pinMode(EINK_SS, OUTPUT);

    pinMode(AMP_POWER_CTRL, OUTPUT);
    digitalWrite(AMP_POWER_CTRL, HIGH);

    ledcSetup(CHANNEL_0, 1000, 8);
    ledcAttachPin(SPEAKER_OUT, CHANNEL_0);
    int i = 3;
    while (i--) {
        ledcWriteTone(CHANNEL_0, 1000);
        delay(200);
        ledcWriteTone(CHANNEL_0, 0);
    }

    button_init();

    SDSPI.begin(SDCARD_CLK, SDCARD_MISO, SDCARD_MOSI);

    if (!SD.begin(SDCARD_SS, SDSPI)) {
        Serial.println("SDCard Mount FAIL");
    } else {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("SDCard %u MB\n", cardSize);
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
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL(gImage_BW, gImage_R); //Refresh the picture in full screen
    EPD_DeepSleep(); //Enter deep sleep,Sleep instruction is necessary, please do not delete!!!
    delay(5000);

    //Clean
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL_Clean();
    EPD_DeepSleep(); //Enter deep sleep,Sleep instruction is necessary, please do not delete!!!

}


void loop()
{
    for (int i = 0; i < sizeof(g_btns) / sizeof(g_btns[0]); ++i) {
        pBtns[i].loop();
    }
}


