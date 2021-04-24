/**
 * Show MPU6050 data
 * Arduino IDE でシリアルモニタを表示している場合は閉じてください
 * シリアルに送信された任意の文字列後の姿勢角度の値をGUI化します
 * 初めにCOMPORTとシリアル通信速度の設定を修正してください
 * 回転軸が異なる場合はrotateX,Y,Zの符号を修正してください
 *
 *
 */
 
import processing.serial.*;

Serial  myPort;
int     lf = 10;       //ASCII linefeed
String  inString;      //String for testing serial communication
int     calibrating;
 
float   dt;
float   x_gyr;  //Gyroscope data
float   y_gyr;
float   z_gyr;
float   x_acc;  //Accelerometer data
float   y_acc;
float   z_acc;
float   x_fil;  //Filtered data
float   y_fil;
float   z_fil;

 
void setup()  { 
//  size(640, 360, P3D); 
  size(1400, 800, P3D);
  noStroke();
  colorMode(RGB, 256); 
 
//  println("in setup");
//  println(Serial.list());
//  println(" Connecting to -> " + Serial.list()[portIndex]);
  myPort = new Serial(this, "COM16", 115200);//same as arduino
  myPort.clear();
  myPort.bufferUntil(lf);
} 

void draw_rect_rainbow() {
  scale(90);
  beginShape(QUADS);

  fill(0, 1, 1); vertex(-1,  1.5,  0.25);
  fill(1, 1, 1); vertex( 1,  1.5,  0.25);
  fill(1, 0, 1); vertex( 1, -1.5,  0.25);
  fill(0, 0, 1); vertex(-1, -1.5,  0.25);

  fill(1, 1, 1); vertex( 1,  1.5,  0.25);
  fill(1, 1, 0); vertex( 1,  1.5, -0.25);
  fill(1, 0, 0); vertex( 1, -1.5, -0.25);
  fill(1, 0, 1); vertex( 1, -1.5,  0.25);

  fill(1, 1, 0); vertex( 1,  1.5, -0.25);
  fill(0, 1, 0); vertex(-1,  1.5, -0.25);
  fill(0, 0, 0); vertex(-1, -1.5, -0.25);
  fill(1, 0, 0); vertex( 1, -1.5, -0.25);

  fill(0, 1, 0); vertex(-1,  1.5, -0.25);
  fill(0, 1, 1); vertex(-1,  1.5,  0.25);
  fill(0, 0, 1); vertex(-1, -1.5,  0.25);
  fill(0, 0, 0); vertex(-1, -1.5, -0.25);

  fill(0, 1, 0); vertex(-1,  1.5, -0.25);
  fill(1, 1, 0); vertex( 1,  1.5, -0.25);
  fill(1, 1, 1); vertex( 1,  1.5,  0.25);
  fill(0, 1, 1); vertex(-1,  1.5,  0.25);

  fill(0, 0, 0); vertex(-1, -1.5, -0.25);
  fill(1, 0, 0); vertex( 1, -1.5, -0.25);
  fill(1, 0, 1); vertex( 1, -1.5,  0.25);
  fill(0, 0, 1); vertex(-1, -1.5,  0.25);

  endShape();
  
  
}

void draw_rect(int r, int g, int b) {
  scale(90);
  beginShape(QUADS);
  
  fill(r, g, b);
  vertex(-1,  1.5,  0.25);
  vertex( 1,  1.5,  0.25);
  vertex( 1, -1.5,  0.25);
  vertex(-1, -1.5,  0.25);

  vertex( 1,  1.5,  0.25);
  vertex( 1,  1.5, -0.25);
  vertex( 1, -1.5, -0.25);
  vertex( 1, -1.5,  0.25);

  vertex( 1,  1.5, -0.25);
  vertex(-1,  1.5, -0.25);
  vertex(-1, -1.5, -0.25);
  vertex( 1, -1.5, -0.25);

  vertex(-1,  1.5, -0.25);
  vertex(-1,  1.5,  0.25);
  vertex(-1, -1.5,  0.25);
  vertex(-1, -1.5, -0.25);

  vertex(-1,  1.5, -0.25);
  vertex( 1,  1.5, -0.25);
  vertex( 1,  1.5,  0.25);
  vertex(-1,  1.5,  0.25);

  vertex(-1, -1.5, -0.25);
  vertex( 1, -1.5, -0.25);
  vertex( 1, -1.5,  0.25);
  vertex(-1, -1.5,  0.25);

  endShape();
  
  
}

void draw()  { 

  background(0);
  lights();
    
  // Tweak the view of the rectangles
  int distance = 50;
  int x_rotation = 90;
  
  //Show gyro data
  pushMatrix(); 
  translate(width/6, height/2, -50); 
  rotateX(radians(-x_gyr - x_rotation));
  rotateY(radians(y_gyr));
  rotateZ(radians(-z_gyr));
  draw_rect(249, 250, 50);
  
  popMatrix(); 

  //Show accel data
  pushMatrix();
  translate(width/2, height/2, -50);
  rotateX(radians(-x_acc - x_rotation));
  rotateY(radians(y_acc));
  rotateZ(radians(-z_acc));
  draw_rect(56, 140, 206);
  popMatrix();
  
  //Show combined data
  pushMatrix();
  translate(5*width/6, height/2, -50);
  rotateX(radians(-y_fil - x_rotation));
  rotateY(radians(x_fil));
  rotateZ(radians(-z_fil));
  draw_rect(93, 175, 83);
  popMatrix();
 
  textSize(24);
  String accStr = "(" + (int) x_acc + ", " + (int) y_acc + ", " + (int) z_acc + ")";
  String gyrStr = "(" + (int) x_gyr + ", " + (int) y_gyr + ", " + (int) z_gyr + ")";
  String filStr = "(" + (int) x_fil + ", " + (int) y_fil + ", " + (int) z_fil + ")";
 

  fill(249, 250, 50);
  text("Gyroscope", (int) width/6.0 - 60, 25);
  text(gyrStr, (int) (width/6.0) - 40, 50);

  fill(56, 140, 206);
  text("Accelerometer", (int) width/2.0 - 50, 25);
  text(accStr, (int) (width/2.0) - 30, 50); 
  
  fill(83, 175, 93);
  text("Combination", (int) (5.0*width/6.0) - 40, 25);
  text(filStr, (int) (5.0*width/6.0) - 20, 50);

} 

void serialEvent(Serial p) {

  inString = (myPort.readString());
  
  try {
    // Parse the data
    String[] dataStrings = split(inString, '#');
    for (int i = 0; i < dataStrings.length; i++) {
      String type = dataStrings[i].substring(0, 4);
      String dataval = dataStrings[i].substring(4);
    if (type.equals("DEL:")) {
        dt = float(dataval);
        /*
        print("Dt:");
        println(dt);
        */
        
      } else if (type.equals("ACC:")) {
        String data[] = split(dataval, ',');
        x_acc = float(data[0]);
        y_acc = float(data[1]);
        z_acc = float(data[2]);
        /*
        print("Acc:");
        print(x_acc);
        print(",");
        print(y_acc);
        print(",");
        println(z_acc);
        */
      } else if (type.equals("GYR:")) {
        String data[] = split(dataval, ',');
        x_gyr = float(data[0]);
        y_gyr = float(data[1]);
        z_gyr = float(data[2]);
      } else if (type.equals("FIL:")) {
        String data[] = split(dataval, ',');
        x_fil = float(data[0]);
        y_fil = float(data[1]);
        z_fil = float(data[2]);
      }
    }
  } catch (Exception e) {
      println("Caught Exception");
  }
}
