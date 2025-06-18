#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define NUM_SENSORS 8
#define THRESHOLD 90

// Pin sensor line follower
const int sensor[NUM_SENSORS] = {2, 1, 0, 3, 5, 7, 6, 4};
int sensorValue[8];
bool bit_Sensor[8];
byte bin_Sensor;
int muxPin[3] = {7, 8, 9};
int error;
int last_error;
int Kp = 10, Ki = 2, Kd = 100;

bool intersection() {
  if (bin_Sensor == 0b10011000 || bin_Sensor == 0b10010000 || bin_Sensor == 0b10001000 || bin_Sensor == 0b10001100 || bin_Sensor == 0b11011000 || bin_Sensor == 0b10000110 || 
      bin_Sensor == 0b00110011 || bin_Sensor == 0b00011011 || bin_Sensor == 0b00001101 || bin_Sensor == 0b00011001 || bin_Sensor == 0b00010001 || bin_Sensor == 0b00001001 || 
      bin_Sensor == 0b10011001 || bin_Sensor == 0b10010001 || bin_Sensor == 0b10001001 || bin_Sensor == 0b10001101 || bin_Sensor == 0b11011011 || bin_Sensor == 0b11010011 || 
      bin_Sensor == 0b11001011 || bin_Sensor == 0b11001101 || bin_Sensor == 0b10000001) return true;
};

const int IN1 = 11;
const int IN2 = 10;
const int IN3 = 5;
const int IN4 = 6;

// Kecepatan motor
const int DEFAULT_SPEED = 75;
const int LOW_SPEED = -75;

// Mode
enum Mode { EXPLORING, SIMPLIFYING_PATH, FASTEST_RUN, FINISHED };
Mode mode = EXPLORING;

// Log path
char exploration_path_log[50];
int path_index = 0;
char shortest_path_actions[50];
int shortest_index = 0;
int current_fastest_path_step = 0;
int kode_belok[10], kode_belok_cepat[10];

void check_Sensor() {
  read_Sensor();
  for (byte i = 0; i < 8; i++) {
    bit_Sensor[i] = (sensorValue[i] > THRESHOLD) ? 1 : 0;
  }

  bin_Sensor = 0b00000000;
  for (int i = 0; i < 8; i++) {
    if (bit_Sensor[sensor[i]]) {
      bin_Sensor |= (1 << (7 - i));
    }
  }
}

void read_Sensor() {
  for (byte pin=0; pin<=7; pin++) {
    selectMuxPin(pin); 
    sensorValue[pin] = analogRead(A0); 
  }
}

void selectMuxPin(byte pin) {
  for (int i=0; i<3; i++) {
    if (pin & (1<<i))
      digitalWrite(muxPin[i], HIGH);
    else
      digitalWrite(muxPin[i], LOW);
  }
}

// Fungsi motor
void setMotor(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    analogWrite(IN1, leftSpeed);
    analogWrite(IN2, 0);
  } else {
    analogWrite(IN1, 0);
    analogWrite(IN2, abs(leftSpeed));
    //leftSpeed = -leftSpeed;
  }

  if (rightSpeed >= 0) {
    analogWrite(IN3, rightSpeed);
    analogWrite(IN4, 0);
  } else {
    analogWrite(IN3, 0);
    analogWrite(IN4, abs(rightSpeed));
    //rightSpeed = -rightSpeed;
  }
}

// Fungsi belok
void turnLeft() {
  delay(150);
  setMotor(0, 0);
  delay(50);
  setMotor(LOW_SPEED, DEFAULT_SPEED);
  Serial.println("Belok Kiri");
  delay(200);
  while(true) {
  check_Sensor();
  if (bin_Sensor == 0b00011000) {
    setMotor(0, 0);
    delay(50);
    break;
  }
  //delay(700);
  }
}

void turnRight() {
  delay(150);
  setMotor(0, 0);
  delay(50);
  setMotor(DEFAULT_SPEED, LOW_SPEED);
  Serial.println("Belok Kanan");
  delay(100);
  while(true) {
  check_Sensor();
  if (bin_Sensor == 0b00011000) {
    setMotor(0, 0);
    delay(50);
    break;
  }
  //delay(700);
  }
}

void uTurn() {
  delay(250);
  setMotor(0, 0);
  delay(50);
  setMotor(-DEFAULT_SPEED, DEFAULT_SPEED);
  Serial.println("Putar Balik");
  while(true) {
  check_Sensor();
  if (bin_Sensor == 0b00011000) {
    setMotor(0, 0);
    delay(50);
    break;
  } //break;
  //delay(950);
  }
}

// Fungsi simplifikasi path
void simplify_path() {
  for (int k=0; k<6; k++) {
    shortest_index = 0; 
    int i = 0;
    while (i < path_index) {
      if (i + 2 < path_index &&
          exploration_path_log[i + 1] == 'U') {
        char a = exploration_path_log[i];
        char b = exploration_path_log[i + 2];

        if ((a == 'L' && b == 'L') || (a == 'R' && b == 'R')) {
          shortest_path_actions[shortest_index] = 'S'; kode_belok_cepat[shortest_index++] = 0; }
        else if ((a == 'L' && b == 'S') || (a == 'S' && b == 'R')) {
          shortest_path_actions[shortest_index] = 'R'; kode_belok_cepat[shortest_index++] = 2; }
        else if ((a == 'S' && b == 'L') || (a == 'R' && b == 'S')) {
          shortest_path_actions[shortest_index] = 'R'; kode_belok_cepat[shortest_index++] = 2; }
        else if ((a == 'S' && b == 'S') || (a == 'L' && b == 'R') || (a == 'R' && b == 'L')) {
          shortest_path_actions[shortest_index] = 'U'; kode_belok_cepat[shortest_index++] = 3; }

        i += 3;
      } else {
        shortest_path_actions[shortest_index] = exploration_path_log[i]; kode_belok_cepat[shortest_index++] = kode_belok[i];
        i++;
      }
    } 
  } 
  path_index = shortest_index;
  for (int l=0; l < shortest_index; l++) {exploration_path_log[l] = shortest_path_actions[l]; kode_belok[l] = kode_belok_cepat[l];}
}

// void simplify_path() {
//   // Salin jalur awal
//   char temp_path[100];          // pastikan ukuran cukup besar
//   int temp_kode[100];

//   for (int i = 0; i < path_index; i++) {
//     temp_path[i] = exploration_path_log[i];
//     temp_kode[i] = kode_belok[i];
//   }

//   int temp_index = path_index;

//   // Loop penyederhanaan berulang
//   bool simplified = true;
//   while (simplified) {
//     simplified = false;
//     shortest_index = 0;

//     int i = 0;
//     while (i < temp_index) {
//       if (i + 2 < temp_index && temp_path[i + 1] == 'U') {
//         char a = temp_path[i];
//         char b = temp_path[i + 2];

//         if ((a == 'L' && b == 'L') || (a == 'R' && b == 'R')) {
//           shortest_path_actions[shortest_index] = 'S';
//           kode_belok_cepat[shortest_index++] = 0;
//         }
//         else if ((a == 'L' && b == 'S') || (a == 'S' && b == 'R')) {
//           shortest_path_actions[shortest_index] = 'R';
//           kode_belok_cepat[shortest_index++] = 2;
//         }
//         else if ((a == 'S' && b == 'L') || (a == 'R' && b == 'S')) {
//           shortest_path_actions[shortest_index] = 'R';
//           kode_belok_cepat[shortest_index++] = 2;
//         }
//         else if ((a == 'S' && b == 'S') || (a == 'L' && b == 'R') || (a == 'R' && b == 'L')) {
//           shortest_path_actions[shortest_index] = 'U';
//           kode_belok_cepat[shortest_index++] = 3;
//         }

//         i += 3;
//         simplified = true; // terjadi penyederhanaan
//       } else {
//         shortest_path_actions[shortest_index] = temp_path[i];
//         kode_belok_cepat[shortest_index++] = temp_kode[i];
//         i++;
//       }
//     }

//     // Salin kembali hasil penyederhanaan untuk dicek lagi
//     for (int j = 0; j < shortest_index; j++) {
//       temp_path[j] = shortest_path_actions[j];
//       temp_kode[j] = kode_belok_cepat[j];
//     }
//     temp_index = shortest_index;
//   }
// }

// void simplify_path() {
//   bool changed;
//   do {
//     changed = false;

//     int new_index = 0;
//     int i = 0;

//     while (i < path_index) {
//       if (i + 2 < path_index &&
//           exploration_path_log[i + 1] == 'U') {
//         char a = exploration_path_log[i];
//         char b = exploration_path_log[i + 2];

//         if ((a == 'L' && b == 'L') || (a == 'R' && b == 'R')) {
//           shortest_path_actions[new_index] = 'S'; kode_belok_cepat[new_index++] = 0;
//         }
//         else if ((a == 'L' && b == 'S') || (a == 'S' && b == 'R')) {
//           shortest_path_actions[new_index] = 'R'; kode_belok_cepat[new_index++] = 2;
//         }
//         else if ((a == 'S' && b == 'L') || (a == 'R' && b == 'S')) {
//           shortest_path_actions[new_index] = 'R'; kode_belok_cepat[new_index++] = 2;
//         }
//         else if ((a == 'S' && b == 'S') || (a == 'L' && b == 'R') || (a == 'R' && b == 'L')) {
//           shortest_path_actions[new_index] = 'U'; kode_belok_cepat[new_index++] = 3;
//         }

//         i += 3; // lewati pola XUX
//         changed = true;
//       } else {
//         shortest_path_actions[new_index] = exploration_path_log[i];
//         kode_belok_cepat[new_index++] = kode_belok[i];
//         i++;
//       }
//     }

//     // Update path_index dan exploration_path_log[] dengan hasil terbaru
//     path_index = new_index;
//     for (int j = 0; j < path_index; j++) {
//       exploration_path_log[j] = shortest_path_actions[j];
//       kode_belok[j] = kode_belok_cepat[j];
//     }

//   } while (changed); // ulangi jika masih ada perubahan
// }


void setup() {
  Serial.begin(9600);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Gagal inisialisasi OLED"));
    //while (true);
  }

  // Inisialisasi pin
  for (int i = 0; i < 3; i++) pinMode(muxPin[i], OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(4, INPUT);
  setMotor(0, 0);

}

void loop() {
  if (mode == EXPLORING) {
    check_Sensor();

    // Deteksi finish line dengan dua kali pendeteksian
    if (bin_Sensor == 0b11111111) {
      delay(10);
      check_Sensor(); 
      if (bin_Sensor == 0b11111111){
      Serial.println("Finish line detected");
      mode = SIMPLIFYING_PATH;
      setMotor(0, 0);
      delay(1000);
      }
    }
    // Navigasi berdasarkan junction
    if (bin_Sensor == 0b00000000) {// Buntu
      exploration_path_log[path_index] = 'U'; kode_belok[path_index++] = 3;
      //delay(250);
      uTurn();
    } 
    else if (bin_Sensor == 0b10000000 || bin_Sensor == 0b11000000 /*|| bin_Sensor == 0b10000110*/) turnLeft();// Belok Kiri
    else if (bin_Sensor == 0b00000001 || bin_Sensor == 0b00000011) turnRight();// Belok Kanan
    else if (bin_Sensor == 0b10011000 || bin_Sensor == 0b10010000 || bin_Sensor == 0b10001000 ||
             bin_Sensor == 0b10001100 || bin_Sensor == 0b11011000 || bin_Sensor == 0b10000110) {//Pertigaan Kiri
        turnLeft();
        exploration_path_log[path_index] = 'L'; kode_belok[path_index++] = 1;
    } else if (bin_Sensor == 0b10000001) {//Pertigaan T
        turnLeft();
        exploration_path_log[path_index] = 'L'; kode_belok[path_index++] = 1;
    } else if (bin_Sensor == 0b10011001 || bin_Sensor == 0b10010001 || bin_Sensor == 0b10001001 || bin_Sensor == 0b10001101 ||
               bin_Sensor == 0b11011011 || bin_Sensor == 0b11010011 || bin_Sensor == 0b11001011 || bin_Sensor == 0b11001101) {//Perempatan
        turnLeft();
        exploration_path_log[path_index] = 'L'; kode_belok[path_index++] = 1;
    } else if (bin_Sensor == 0b00110011 || bin_Sensor == 0b00011011 || bin_Sensor == 0b00001101 ||
               bin_Sensor == 0b00011001 || bin_Sensor == 0b00010001 || bin_Sensor == 0b00001001) {
      delay(150);
      exploration_path_log[path_index] = 'S'; kode_belok[path_index++] = 0;
      /*check_Sensor();
      if (bin_Sensor == 0b00011001 || bin_Sensor == 0b00010001 || bin_Sensor == 0b00001001 || bin_Sensor == 0b00001101
               //bin_Sensor == 0b00011010 || bin_Sensor == 0b00011011) {Pertigaan Kanan
      //calculateError(); delay(100);
      //exploration_path_log[path_index++] = 'S';}*/
    } else calculateError();
        //setMotor(PWM_R, PWM_L);

  } else if (mode == SIMPLIFYING_PATH) {
    Serial.println("Simplifying path...");
    setMotor(0, 0);
     display.clearDisplay();
     display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // teks putih, latar hitam
     display.setCursor(0, 0);
     display.setTextSize(1);
     display.println(exploration_path_log);
    // display.display();
    //  for (int i=0; i<6; i++) {
    simplify_path();
    //   exploration_path_log = shortest_path_actions;
    //  }
     //display.setCursor(0, 32);
     display.println(shortest_path_actions);
     Serial.println(shortest_path_actions);
     display.display();

    mode = FASTEST_RUN;
    delay(5000);
  } else if (mode == FASTEST_RUN) {
    while (true) {
      // if (current_fastest_path_step >= 3) {
      //     mode = FINISHED;
      //     setMotor(0, 0);
      //     // display.clearDisplay();
      //     // display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // teks putih, latar hitam
      //     // display.setCursor(0, 0);
      //     // display.setTextSize(2);
      //      Serial.println("Selesai");
      //     // display.display();
      //     break;
      // }
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK); // teks putih, latar hitam
      display.setCursor(0, 0);
      display.setTextSize(2);
      display.println(current_fastest_path_step);
      // display.setCursor(0, 32);
      // display.setTextSize(2);
      display.print("aksi: ");
      display.println(shortest_path_actions[current_fastest_path_step]);
      display.display();
      check_Sensor();
      if (bin_Sensor == 0b10011000 || bin_Sensor == 0b10010000 || bin_Sensor == 0b10001000 || bin_Sensor == 0b10001100 || bin_Sensor == 0b11011000 || bin_Sensor == 0b10000110 || 
          bin_Sensor == 0b00110011 || bin_Sensor == 0b00011011 || bin_Sensor == 0b00001101 || bin_Sensor == 0b00011001 || bin_Sensor == 0b00010001 || bin_Sensor == 0b00001001 || 
          bin_Sensor == 0b10011001 || bin_Sensor == 0b10010001 || bin_Sensor == 0b10001001 || bin_Sensor == 0b10001101 || bin_Sensor == 0b11011011 || bin_Sensor == 0b11010011 || 
          bin_Sensor == 0b11001011 || bin_Sensor == 0b11001101 || bin_Sensor == 0b10000001) {
          Serial.print("aksi sekarang: ");
          Serial.println(shortest_path_actions[current_fastest_path_step]);
          if (kode_belok_cepat[current_fastest_path_step] == 1/*shortest_path_actions[current_fastest_path_step] == 'L'*/) turnLeft();
          else if (kode_belok_cepat[current_fastest_path_step] == 2/*shortest_path_actions[current_fastest_path_step] == 'R'*/) turnRight();
          else if (kode_belok_cepat[current_fastest_path_step] == 3/*shortest_path_actions[current_fastest_path_step] == 'U'*/) uTurn();
          else calculateError(); delay(150);
          current_fastest_path_step++;
      }
      else if (bin_Sensor == 0b10000000 || bin_Sensor == 0b11000000 /*|| bin_Sensor == 0b10000110*/) turnLeft();// Belok Kiri
      else if (bin_Sensor == 0b00000001 || bin_Sensor == 0b00000011) turnRight();// Belok Kanan
      else if (bin_Sensor == 0b11111111) setMotor(0, 0);
      else calculateError(); //Lurus Line Following
      
    }

  } else if (mode == FINISHED) {
    setMotor(0, 0);
    delay(100);
  }
}

      void calculateError() {
        if (bin_Sensor == 0b00011000) error = 0;
        if (bin_Sensor == 0b00010000) error = -1;
        if (bin_Sensor == 0b00110000) error = -3;
        if (bin_Sensor == 0b00100000) error = -5;
        if (bin_Sensor == 0b01100000) error = -7;
        if (bin_Sensor == 0b01000000) error = -9;
        if (bin_Sensor == 0b00001000) error = 1;
        if (bin_Sensor == 0b00001100) error = 3;
        if (bin_Sensor == 0b00000100) error = 5;
        if (bin_Sensor == 0b00000110) error = 7;
        if (bin_Sensor == 0b00000010) error = 9;

        /*int P = error;
        int I = I + error;
        int D = error - last_error;
        int PID = (Kp * P) + (Ki * I) + (Kd * D);*/ 
        //........................................

        //........................................Applying the result of PID calculation to the driving motors.
        int PWM_R = DEFAULT_SPEED - 2*error;//- PID;
        int PWM_L = DEFAULT_SPEED + 2*error;//+ PID;
        //PWM_R = PWM_R - PID;
        //PWM_L = PWM_L + PID; 

        //last_error = error;

        /*if (PWM_R >= 0) {
          right_Forward();
        } else {
          right_Backward();
        }

        if (PWM_L >= 0) {
          left_Forward();
        } else {
          left_Backward();
        }

        if (PWM_R < 0) PWM_R = 0 - PWM_R;
        if (PWM_L < 0) PWM_L = 0 - PWM_L;
        if (PWM_R > 255) PWM_R = 255;
        if (PWM_L > 255) PWM_L = 255;

        if (digitalRead(6) == LOW) {
          PWM_R = abs(PWM_R);
        } else {
          PWM_R = 255 - PWM_R;
        }

        if (digitalRead(10) == LOW) {
          PWM_L = abs(PWM_L);
        } else {
          PWM_L = 255 - PWM_L;
        }*/

        setMotor(PWM_L, PWM_R);
        Serial.println("Lurus");
        //analogWrite(11, PWM_R);
        //analogWrite(5, PWM_L);
      }
