#include "SD.h"
#include "SPI.h"
#include "picture.h"

#define MONO 1
#define RED  2

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

#define BUTTON_1        37
#define BUTTON_2        38
#define BUTTON_3        39

#define SPEAKER_OUT     25
#define AMP_POWER_CTRL  19
#define CHANNEL_0       0

#define POS 1
#define NEG 2

#define MAX_LINE_BYTES          16// =128/8
#define MAX_COLUMN_BYTES        296
#define ALLSCREEN_GRAGHBYTES    4736

SPIClass SDSPI(VSPI);

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


    Epaper_Write_Command(0x74); //set analog block control
    Epaper_Write_Data(0x54);
    Epaper_Write_Command(0x7E); //set digital block control
    Epaper_Write_Data(0x3B);

    Epaper_Write_Command(0x01); //Driver output control
    Epaper_Write_Data(0x27);
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x00);

    Epaper_Write_Command(0x11); //data entry mode
    Epaper_Write_Data(0x01);

    Epaper_Write_Command(0x44); //set Ram-X address start/end position
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x10);    //0x0C-->(15+1)*8=128

    Epaper_Write_Command(0x45); //set Ram-Y address start/end position
    Epaper_Write_Data(0x27);   //0xF9-->(249+1)=250
    Epaper_Write_Data(0x01);
    Epaper_Write_Data(0x00);
    Epaper_Write_Data(0x00);

    Epaper_Write_Command(0x3C); //BorderWavefrom
    Epaper_Write_Data(0x01);

    Epaper_Write_Command(0x18);
    Epaper_Write_Data(0x80);

    Epaper_Write_Command(0x22); // //Load Temperature and waveform setting.
    Epaper_Write_Data(0XB1);
    Epaper_Write_Command(0x20);

    Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
    Epaper_Write_Data(0x01);
    Epaper_Write_Command(0x4F);   // set RAM y address count to 0X127;
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
void EPD_ALL_image(const unsigned char *datas1, const unsigned char *datas2)
{
    unsigned int i;
    Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(*datas1);
        datas1++;
    }
    Epaper_Write_Command(0x26);   //write RAM for black(0)/white (1)
    for (i = 0; i < ALLSCREEN_GRAGHBYTES; i++) {
        Epaper_Write_Data(*datas2);
        datas2++;
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


///////////////////////////Part update//////////////////////////////////////////////
void EPD_Dis_Part(unsigned int x_start, unsigned int y_start, const unsigned char *datas, const unsigned char color_mode, unsigned int PART_COLUMN, unsigned int PART_LINE)
{
    unsigned int i;
    unsigned int x_end, y_start1, y_start2, y_end1, y_end2;

    x_start = (x_start + 8) / 8; //转换为字节
    x_end = x_start + PART_LINE / 8 - 1;

    y_start1 = 0;
    y_start2 = y_start;
    if (y_start >= 256) {
        y_start1 = y_start2 / 256;
        y_start2 = y_start2 % 256;
    }
    y_end1 = 0;
    y_end2 = y_start + PART_COLUMN - 1;
    if (y_end2 >= 256) {
        y_end1 = y_end2 / 256;
        y_end2 = y_end2 % 256;
    }

    Epaper_Write_Command(0x44);       // set RAM x address start/end, in page 35
    Epaper_Write_Data(x_start);         // RAM x address start
    Epaper_Write_Data(x_end);               // RAM x address end
    Epaper_Write_Command(0x45);       // set RAM y address start/end, in page 35
    Epaper_Write_Data(y_start2);        // RAM y address start
    Epaper_Write_Data(y_start1);        // RAM y address start
    Epaper_Write_Data(y_end2);          // RAM y address end
    Epaper_Write_Data(y_end1);          // RAM y address end


    Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
    Epaper_Write_Data(x_start);
    Epaper_Write_Command(0x4F);   // set RAM y address count to 0X127;
    Epaper_Write_Data(y_start2);
    Epaper_Write_Data(y_start1);

    if (color_mode == MONO)
        Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)

    if (color_mode == RED)
        Epaper_Write_Command(0x26);   //write RAM for black(0)/white (1)

    for (i = 0; i < PART_COLUMN * PART_LINE / 8; i++) {
        Epaper_Write_Data(* datas);
        datas++;
    }

    EPD_Update();

}


void EPD_Dis_Part_mult(unsigned int x_startA, unsigned int y_startA, const unsigned char *datasA1, const unsigned char *datasA2,
                       unsigned int x_startB, unsigned int y_startB, const unsigned char *datasB1, const unsigned char *datasB2,
                       unsigned int PART_COLUMN, unsigned int PART_LINE)
{
    unsigned int i;
    unsigned int x_endA, y_startA1, y_startA2, y_endA1, y_endA2;
    unsigned int x_endB, y_startB1, y_startB2, y_endB1, y_endB2;

    //Data A////////////////////////////
    x_startA = (x_startA + 8) / 8; //转换为字节
    x_endA = x_startA + PART_LINE / 8 - 1;

    y_startA1 = 0;
    y_startA2 = y_startA;
    if (y_startA2 >= 256) {
        y_startA1 = y_startA2 / 256;
        y_startA2 = y_startA2 % 256;
    }
    y_endA1 = 0;
    y_endA2 = y_startA + PART_COLUMN - 1;
    if (y_endA2 >= 256) {
        y_endA1 = y_endA2 / 256;
        y_endA2 = y_endA2 % 256;
    }

    Epaper_Write_Command(0x44);       // set RAM x address start/end, in page 35
    Epaper_Write_Data(x_startA);    // RAM x address start at 00h;
    Epaper_Write_Data(x_endA);    // RAM x address end at 0fh(15+1)*8->128
    Epaper_Write_Command(0x45);       // set RAM y address start/end, in page 35
    Epaper_Write_Data(y_startA2);    // RAM y address start at 0127h;
    Epaper_Write_Data(y_startA1);    // RAM y address start at 0127h;
    Epaper_Write_Data(y_endA2);    // RAM y address end at 00h;
    Epaper_Write_Data(y_endA1);

    Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
    Epaper_Write_Data(x_startA);
    Epaper_Write_Command(0x4F);   // set RAM y address count to 0X127;
    Epaper_Write_Data(y_startA2);
    Epaper_Write_Data(y_startA1);

    Epaper_Write_Command(0x24);   //Write Black and White image to RAM
    for (i = 0; i < PART_COLUMN * PART_LINE / 8; i++) {
        Epaper_Write_Data(*datasA1);
        datasA1++;
    }

    Epaper_Write_Command(0x26);   //Write Black and White image to RAM
    for (i = 0; i < PART_COLUMN * PART_LINE / 8; i++) {
        Epaper_Write_Data(*datasA2);
        datasA2++;
    }

    //Data B/////////////////////////////////////
    x_startB = (x_startB + 8) / 8; //转换为字节
    x_endB = x_startB + PART_LINE / 8 - 1;

    y_startB1 = 0;
    y_startB2 = y_startB;
    if (y_startB2 >= 256) {
        y_startB1 = y_startB2 / 256;
        y_startB2 = y_startB2 % 256;
    }
    y_endB1 = 0;
    y_endB2 = y_startB + PART_COLUMN - 1;
    if (y_endB2 >= 256) {
        y_endB1 = y_endB2 / 256;
        y_endB2 = y_endB2 % 256;
    }

    Epaper_Write_Command(0x44);       // set RAM x address start/end, in page 35
    Epaper_Write_Data(x_startB);    // RAM x address start at 00h;
    Epaper_Write_Data(x_endB);    // RAM x address end at 0fh(15+1)*8->128
    Epaper_Write_Command(0x45);       // set RAM y address start/end, in page 35
    Epaper_Write_Data(y_startB2);    // RAM y address start at 0127h;
    Epaper_Write_Data(y_startB1);    // RAM y address start at 0127h;
    Epaper_Write_Data(y_endB2);    // RAM y address end at 00h;
    Epaper_Write_Data(y_endB1);

    Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
    Epaper_Write_Data(x_startB);
    Epaper_Write_Command(0x4F);   // set RAM y address count to 0X127;
    Epaper_Write_Data(y_startB2);
    Epaper_Write_Data(y_startB1);

    Epaper_Write_Command(0x24);   //Write Black and White image to RAM
    for (i = 0; i < PART_COLUMN * PART_LINE / 8; i++) {
        Epaper_Write_Data(*datasB1);
        datasB1++;
    }

    Epaper_Write_Command(0x26);   //Write Black and White image to RAM
    for (i = 0; i < PART_COLUMN * PART_LINE / 8; i++) {
        Epaper_Write_Data(*datasB2);
        datasB2++;
    }

    EPD_Update();

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

    SPI.begin(SPI_CLK, SPI_MISO, SPI_MOSI);
    SDSPI.begin(SDCARD_CLK, SDCARD_MISO, SDCARD_MOSI, SDCARD_SS);

    if (SD.begin(SDCARD_SS, SDSPI)) {
        uint32_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.println("SDCard:" + String(cardSize) + "MB");
    } else {
        Serial.println("SDCard  None");
    }

    EPD_HW_Init();
    EPD_ALL_image(gImage_1_BW, gImage_1_RED);   //Refresh the picture in full screen
    EPD_DeepSleep();
    delay(3000);

    EPD_HW_Init();
    EPD_ALL_image(gImage_2_BW, gImage_2_RED);   //Refresh the picture in full screen
    EPD_DeepSleep();
    delay(3000);

    //Full screen refresh
    EPD_HW_Init();                                                  //Electronic paper initialization
    EPD_ALL_image(gImage_basemap_BW, gImage_basemap_RED);           //Refresh the picture in full screen
    EPD_DeepSleep();
    delay(3000);

    EPD_HW_Init();                                                  //Electronic paper initialization
    delay(500);
//////////////////////Partial refresh digital presentation///////////////////
    uint8_t m, k;
    for (k = 0; k < 2; k++) {
        m = k * 2;
        EPD_HW_Init(); //Electronic paper initialization
        EPD_Dis_Part(32, 231, Num_MONO[m], MONO, 56, 56); //x,y,DATA,Resolution 56*56
        EPD_DeepSleep();  //Enter deep sleep
        delay(1000);

        EPD_HW_Init(); //Electronic paper initialization
        EPD_Dis_Part(32, 175, Num_RED[m + 1], RED, 56, 56); //x,y,DATA,Resolution 56*56
        EPD_DeepSleep();  //Enter deep sleep
        delay(1000);
    }

    for (k = 0; k < 2; k++) {
        m = k * 2;
        EPD_HW_Init(); //Electronic paper initialization
        EPD_Dis_Part_mult(32, 231, ABC_MONO[m], ABC_RED[m], //x,y,DATA,Resolution 56*56
                          32, 175, ABC_MONO[m + 1], ABC_RED[m + 1],
                          56, 56);
        EPD_DeepSleep();  //Enter deep sleep
        delay(1000);
    }

    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_White();  //Show all white
    EPD_DeepSleep();  //Enter deep sleep
    //Enter deep sleep
    delay(1500);
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
