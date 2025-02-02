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

#include "MM-ToF10.h"     // ToF Library (https://github.com/Interested-In-Spresense/MM-ToF10)
#include <Camera.h>
#include <SPI.h>
#include <EEPROM.h>
#include <DNNRT.h>
#include "Adafruit_ILI9341.h"
#include <MP.h>
#include <SDHCI.h>

SDClass theSD;

/* LCD Settings */
#define TFT_CS        -1
#define TFT_RST       8
#define TFT_DC        9

/* 画像サイズ設定 */
#define DNN_IMG_W     56
#define DNN_IMG_H     56
#define CAM_IMG_W     320
#define CAM_IMG_H     240
#define CAM_CLIP_X    48
#define CAM_CLIP_Y    0
#define CAM_CLIP_W    224
#define CAM_CLIP_H    224

#define LINE_THICKNESS 5
#define BAUDRATE      115200
#define THRESHOLD     1000.0f  // 1000 [mm]

// LCD オブジェクト
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

// DNNRT関係
DNNRT dnnrt;
DNNVariable input(3 * DNN_IMG_W * DNN_IMG_H);  // RGB3チャネル

// RGB565の色成分正規化用（各成分の最大値の逆数）
static constexpr float inv31 = 1.0f / 31.0f;
static constexpr float inv63 = 1.0f / 63.0f;
static const uint8_t label[2] = { 0, 1 };

// センサおよび推論結果のフラグ
bool d_flag = false;  // ToFセンサの閾値チェック用
bool c_flag = false;  // DNN推論結果用

// ToFセンサから取得する8×4の距離データ（[mm]）
static float data[8][4];
float* ptr = (float*)data;

// カメラエラー表示
void printError(CamErr err) {
  Serial.print("Error: ");
  switch (err) {
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
      Serial.println("Unknown error");
      break;
  }
}

// LCDに文字列を表示する
void putStringOnLcd(const String& str, int color) {
  tft.fillRect(0, 224, 320, 240, ILI9341_BLACK);
  tft.setTextSize(2);
  int sx = 160 - (str.length() * 6); // 文字幅約12/2=6
  if (sx < 0) sx = 0;
  tft.setCursor(sx, 225);
  tft.setTextColor(color);
  tft.println(str);
}

// 画像に枠を描画する
void drawBox(uint16_t* imgBuf, int color) {
  // 上下の枠線
  for (int x = CAM_CLIP_X; x < CAM_CLIP_X + CAM_CLIP_W; ++x) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      imgBuf[CAM_IMG_W * (CAM_CLIP_Y + n) + x] = color;
      imgBuf[CAM_IMG_W * (CAM_CLIP_Y + CAM_CLIP_H - 1 - n) + x] = color;
    }
  }
  // 左右の枠線
  for (int y = CAM_CLIP_Y; y < CAM_CLIP_Y + CAM_CLIP_H; ++y) {
    for (int n = 0; n < LINE_THICKNESS; ++n) {
      imgBuf[CAM_IMG_W * y + CAM_CLIP_X + n] = color;
      imgBuf[CAM_IMG_W * y + CAM_CLIP_X + CAM_CLIP_W - 1 - n] = color;
    }
  }
}

// カメラ画像取得時のコールバック（画像取得＋ToF処理＋DNN推論）
void CamCB(CamImage img) {
  d_flag = false;
  c_flag = false;
  int str_color = ILI9341_GREEN;

  if (!img.isAvailable()) {
    Serial.println("Image is not available. Try again");
    return;
  }

  // ToFセンサから3Dデータ取得
  MMToF10.get3d(ptr);
  for (int j = 0; j < 8; j++) {
    for (int i = 0; i < 4; i++) {
      float distance = data[j][i];
      printf("range[%d][%d] = %f [mm]\n", j, i, distance);
      if (distance < THRESHOLD) {
        d_flag = true;
      }
    }
  }
  if (d_flag) {
    printf("センサの距離が閾値(%f [mm])未満の値を検出\n", THRESHOLD);
  }

  // 画像のクリップ＆リサイズ（DNN入力サイズへ変換）
  CamImage small;
  CamErr err = img.clipAndResizeImageByHW(
    small,
    CAM_CLIP_X, CAM_CLIP_Y,
    CAM_CLIP_X + CAM_CLIP_W - 1,
    CAM_CLIP_Y + CAM_CLIP_H - 1,
    DNN_IMG_W, DNN_IMG_H);
  if (!small.isAvailable()) {
    putStringOnLcd("Clip and Resize Error: " + String(err), ILI9341_RED);
    return;
  }

  // 画像をRGB565に変換
  small.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* temp = (uint16_t*)small.getImgBuff();

  // DNNRT入力用のデータ準備（各チャネルごとに正規化）
  float* r = input.data();
  float* g = r + DNN_IMG_W * DNN_IMG_H;
  float* b = g + DNN_IMG_W * DNN_IMG_H;
  for (int i = 0; i < DNN_IMG_W * DNN_IMG_H; ++i) {
    uint16_t pixel = *temp++;
    *r++ = ((pixel >> 11) & 0x1F) * inv31;
    *g++ = ((pixel >> 5) & 0x3F) * inv63;
    *b++ = (pixel & 0x1F) * inv31;
  }

  // DNN推論実行
  dnnrt.inputVariable(input, 0);
  dnnrt.forward();
  DNNVariable output = dnnrt.outputVariable(0);
  int index = output.maxIndex();

  // 推論結果に応じた処理（0.75を閾値として判定）
  if (output[1] > 0.75f) {
    c_flag = true;
    str_color = ILI9341_RED;
  }
  String gStrResult;
  if (index < 2) {
    gStrResult = String(label[index]) + ":" + String(output[index]);
  } else {
    gStrResult = String("?:") + String(output[index]);
  }
  Serial.println(gStrResult);

  // キャプチャ画像のフォーマットをRGB565に変換
  img.convertPixFormat(CAM_IMAGE_PIX_FMT_RGB565);
  uint16_t* imgBuf = (uint16_t*)img.getImgBuff();

  if (d_flag || c_flag) {
    Serial.println("L:0,R:0");  // RP2040 BLDC Driver Stop
    drawBox(imgBuf, ILI9341_RED);
  } else {
    // BLDC Continue oparation
    drawBox(imgBuf, ILI9341_GREEN);
  }

  // LCDへ画像と推論結果の描画
  tft.drawRGBBitmap(0, 0, imgBuf, 320, 224);
  putStringOnLcd(gStrResult, str_color);

  // メモリ使用状況のログ出力
  int usedMem, freeMem, largestFreeMem;
  MP.GetMemoryInfo(usedMem, freeMem, largestFreeMem);
  MPLog("Used:%4d [KB] / Free:%4d [KB] (Largest:%4d [KB])\n",
        usedMem / 1024, freeMem / 1024, largestFreeMem / 1024);
}

void setup() {
  Serial.begin(BAUDRATE);
  while (!Serial) {
    ; // USB接続待ち
  }

  tft.begin();
  tft.setRotation(3);

  MMToF10.begin();
  MMToF10.sync();
  MMToF10.nomal(ShortDistance, LowSpeed);

  // SDカードの初期化
  while (!theSD.begin()) {
    putStringOnLcd("Insert SD card", ILI9341_RED);
  }

  // SDカードからモデルファイル(model.nnb)をオープン
  File nnbfile = theSD.open("model.nnb", FILE_READ);
  if (!nnbfile) {
    Serial.println("model.nnbファイルを開けませんでした");
    return;
  }
  Serial.println("model.nnbファイルを開きました");

  // ファイルポインタを先頭に戻す
  if (!nnbfile.seek(0)) {
    Serial.println("ファイルポインタの先頭への移動に失敗しました");
  }

  // モデルファイルサイズの確認
  size_t fileSize = nnbfile.size();
  Serial.print("ファイルサイズ: ");
  Serial.println(fileSize);
  if (fileSize == 0) {
    Serial.println("ファイルサイズが0です。ファイル破損が無いか確認してください。");
    nnbfile.close();
    return;
  }

  // DNNRT初期化
  int ret = dnnrt.begin(nnbfile);
  if (ret < 0) {
    putStringOnLcd("dnnrt.begin failed: " + String(ret), ILI9341_RED);
    return;
  }

  // HDRカメラ初期化＆ストリーミング開始
  theCamera.begin();
  Serial.println("Set still picture format");
  CamErr err = theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_QQVGA_H, CAM_IMGSIZE_QQVGA_V,
    CAM_IMAGE_PIX_FMT_JPG,
    11
  );

  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }
  
  Serial.println("Set Auto white balance parameter");
  err = theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_DAYLIGHT);
  if (err != CAM_ERR_SUCCESS) {
    printError(err);
  }
  
  theCamera.startStreaming(true, CamCB);
}

void loop() {}