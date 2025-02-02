/*
 *  camera.ino - Simple camera example sketch
 *  Copyright 2018, 2022 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *  This is a test app for the camera library.
 *  This library can only be used on the Spresense with the FCBGA chip package.
 */

#include "MM-ToF10.h" // ToF Library developed by Interested-In-Spresense https://github.com/Interested-In-Spresense/MM-ToF10
#include <Camera.h>
#include <SPI.h>
#include <EEPROM.h>
#include <DNNRT.h>
#include "Adafruit_ILI9341.h"
#include <MP.h>
#include <SDHCI.h>
SDClass theSD;

/* LCD Settings */
#define TFT_CS -1
#define TFT_RST 8
#define TFT_DC 9

#define DNN_IMG_W 28
#define DNN_IMG_H 28
#define CAM_IMG_W 320
#define CAM_IMG_H 240
#define CAM_CLIP_X 48
#define CAM_CLIP_Y 0
#define CAM_CLIP_W 224
#define CAM_CLIP_H 224

#define LINE_THICKNESS (5)
#define BAUDRATE (115200)
#define THRESHOLD 500.0f  //500mm

Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

DNNRT dnnrt;
DNNVariable input(3 * DNN_IMG_W * DNN_IMG_H);

// RGB565->色成分変換の逆数を事前に計算
const float inv31 = 1.0f / 31.0f;
const float inv63 = 1.0f / 63.0f;
static uint8_t const label[2] = { 0, 1 };
bool d_flag = false;
static float data[8][4];
float* ptr = (float*)data;

// カメラエラーを表示する関数
void printError(enum CamErr err)
{
  Serial.print("Error: ");
  switch (err)
    {
      case CAM_ERR_NO_DEVICE:
        Serial.println("No Device");
        break;
      case CAM_ERR_ILLEGAL_DEVERR:
        Serial.println("Illegal device error");
        break;
      case CAM_ERR_ALREADY_INITIALIZED:
        Serial.println("Already initialized");
        break;
      case CAM_ERR_NOT_INITIALIZED:
        Serial.println("Not initialized");
        break;
      case CAM_ERR_NOT_STILL_INITIALIZED:
        Serial.println("Still picture not initialized");
        break;
      case CAM_ERR_CANT_CREATE_THREAD:
        Serial.println("Failed to create thread");
        break;
      case CAM_ERR_INVALID_PARAM:
        Serial.println("Invalid parameter");
        break;
      case CAM_ERR_NO_MEMORY:
        Serial.println("No memory");
        break;
      case CAM_ERR_USR_INUSED:
        Serial.println("Buffer already in use");
        break;
      case CAM_ERR_NOT_PERMITTED:
        Serial.println("Operation not permitted");
        break;
      default:
        break;
    }
}

// LCDに文字列を表示する関数
void putStringOnLcd(String str, int color) {
  int len = str.length();
  tft.fillRect(0, 224, 320, 240, ILI9341_BLACK);
  tft.setTextSize(2);
  int sx = 160 - len / 2 * 12;
  if (sx < 0) sx = 0;
  tft.setCursor(sx, 225);
  tft.setTextColor(color);
  tft.println(str);
}

// 画像に枠を描画する関数
void drawBox(uint16_t* imgBuf) {
  /* Draw target line */
  for (int x = CAM_CLIP_X; x < CAM_CLIP_X + CAM_CLIP_W; ++x) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      *(imgBuf + CAM_IMG_W * (CAM_CLIP_Y + n) + x) = ILI9341_RED;
      *(imgBuf + CAM_IMG_W * (CAM_CLIP_Y + CAM_CLIP_H - 1 - n) + x) = ILI9341_RED;
    }
  }
  for (int y = CAM_CLIP_Y; y < CAM_CLIP_Y + CAM_CLIP_H; ++y) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      *(imgBuf + CAM_IMG_W * y + CAM_CLIP_X + n) = ILI9341_RED;
      *(imgBuf + CAM_IMG_W * y + CAM_CLIP_X + CAM_CLIP_W - 1 - n) = ILI9341_RED;
    }
  }
}

// カメラ画像取得時のコールバック関数+ToF
void CamCB(CamImage img) {
  if (!img.isAvailable()) {
    Serial.println("Image is not available. Try again");
    return;
  }

  // 3D ToFセンサ(MM-TOF10-IS)からデータを取得
  d_flag = false;
  MMToF10.get3d(ptr);

  for (int j = 0; j < 8; j++){
    for (int i = 0; i < 4; i++){
      float distance = data[j][i];
      printf("range[%d][%d] = %f [mm]\n", j, i, distance);
      if(distance < THRESHOLD) {
        d_flag = true;
      }
    }
  }
  
  if(d_flag){
    printf("センサの距離が閾値(%f [mm])未満の値を検出\n", THRESHOLD);
  }

  // 画像をクリップしてリサイズ
  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(small, CAM_CLIP_X, CAM_CLIP_Y, CAM_CLIP_X + CAM_CLIP_W - 1, CAM_CLIP_Y + CAM_CLIP_H - 1, DNN_IMG_W, DNN_IMG_H);
  if (!small.isAvailable()) {
    putStringOnLcd("Clip and Reize Error:" + String(err), ILI9341_RED);
    return;
  }

  // 画像フォーマットをRGB565に変換
  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* temp = (uint16_t*)small.getImgBuff();

  // DNNRTに入力するためのデータを準備
  float* r = input.data();
  float* g = r + DNN_IMG_W * DNN_IMG_H;
  float* b = g + DNN_IMG_W * DNN_IMG_H;
  for (int i = 0; i < DNN_IMG_W * DNN_IMG_H; ++i) {
    uint16_t pixel = *temp++;
    *r++ = ((pixel >> 11) & 0x1F) * inv31;
    *g++ = ((pixel >> 5) & 0x3F) * inv63;
    *b++ = (pixel & 0x1F) * inv31;
  }

  // DNNRTで推論を実行
  String gStrResult = "?";
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();

  DNNVariable output = dnnrt.outputVariable(0);
  int index = output.maxIndex();
  if (output[1] > 0.75 || d_flag) {
    Serial.println("L:0,R:0");  //RP2040 BLDC Driver Stop
  } else {
    //continue move BLDC
  }

  // 推論結果を表示
  if (index < 2) {
    gStrResult = String(label[index]) + String(":") + String(output[index]);
  } else {
    gStrResult = String("?:") + String(output[index]);
  }
  Serial.println(gStrResult);

  // 画像をRGB565フォーマットに変換
  img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* imgBuf = (uint16_t*)img.getImgBuff();

  // 画像に枠を描画
  drawBox(imgBuf);
  tft.drawRGBBitmap(0, 0, (uint16_t*)img.getImgBuff(), 320, 224);
  putStringOnLcd(gStrResult, ILI9341_YELLOW);

  int usedMem, freeMem, largestFreeMem;
  MP.GetMemoryInfo(usedMem, freeMem, largestFreeMem);
  MPLog("Used:%4d [KB] / Free:%4d [KB] (Largest:%4d [KB])\n",
  usedMem / 1024, freeMem / 1024, largestFreeMem / 1024);
}

// 初期設定
void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial)
    {
      ; /* wait for serial port to connect. Needed for native USB port only */
    }
  tft.begin();
  tft.setRotation(3);
  MMToF10.begin();
  MMToF10.sync();
  MMToF10.nomal(ShortDistance,LowSpeed);

  CamErr err;

  // SDカードの初期化
  while (!theSD.begin()) { putStringOnLcd("Insert SD card", ILI9341_RED); }

  // SDカードからモデルファイルを開く
  File nnbfile = theSD.open("model.nnb", FILE_READ);
  if (!nnbfile) {
    Serial.println("model.nnbファイルを開けませんでした");
    return;
  }
  Serial.println("model.nnbファイルをオープンしました");

  // 念のためファイルポインタを先頭に戻す
  if (!nnbfile.seek(0)) {
    Serial.println("ファイルポインタの先頭への移動に失敗しました");
  }

  // ファイルサイズのチェック
  size_t fileSize = nnbfile.size();
  Serial.print("ファイルサイズ: ");
  Serial.println(fileSize);
  if (fileSize == 0) {
    Serial.println("注意: ファイルサイズが0です。ファイルにデータがあるか確認してください。");
    nnbfile.close();
    return;
  }
  // if (fileSize > SHM_SIZE) {
  //   Serial.println("model.nnbファイルが大きすぎます");
  //   nnbfile.close();
  //   return;
  // }

  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    putStringOnLcd("dnnrt.begin failed" + String(ret), ILI9341_RED);
    return;
  }

  // HDRカメラ(CXD5602PWBCAM2W)の初期化とストリーミング開始
  theCamera.begin();

  // 静止画フォーマットを設定
  Serial.println("Set still picture format");
  // https://developer.sony.com/spresense/spresense-api-references-arduino/group__CAM__IMGSIZE.html
  // err = theCamera.setStillPictureImageFormat(CAM_IMGSIZE_QUADVGA_H, CAM_IMGSIZE_QUADVGA_V, CAM_IMAGE_PIX_FMT_JPG, 11);
  err = theCamera.setStillPictureImageFormat(CAM_IMGSIZE_QQVGA_H, CAM_IMGSIZE_QQVGA_V, CAM_IMAGE_PIX_FMT_JPG, 11);
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }

  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS)
    {
      printError(err);
    }
  
  theCamera.startStreaming(true, CamCB);
}

void loop() {}