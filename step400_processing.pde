/**
 * Step motor driver Step400 demo project using Andreas Schlegel's oscP5 library
 * written by Yuske Goto https://yuskegoto.net
 * 2020.12.19
 */
 
import oscP5.*;
import netP5.*;

OscP5 oscP5;
NetAddress step400Address;

/****************** Network setting ************************/
int INCOMING_PORT = 50101;
int OUTGOING_PORT = 50000;
String DRIVER_IP = "10.0.0.101";

/****************** Motor basic setting ************************/
int MOTOR_NO = 1;
int K_HOLD = 20;
int K_RUN = 169;
int K_ACC = 169;
int K_DEC = 169;
float RUN_MAX = 650;
// 12800 for one rotation
float GOTO_MAX = 12800/2;
String currentState;
float currentValRun = 0;
float currentValGoTo = 0;

/****************** Servo PID setting ************************/
float currentValPid = 0;
float SERVO_KP = 0.06;
float SERVO_KI = 0;
float SERVO_KD = 0;

/****************** UI setting ************************/
int BUTTON_SETIP[] = {0, 0, 199, 99};
int BUTTON_INIT[] = {200, 0, 199, 99};
int BUTTON_STOP[] = {400, 0, 200, 99};
int BUTTON_RUN[] = {500, 100, 100, 199};
int FIELD_RUN[] = {0, 100, 500, 200};
int BUTTON_GOTO[] = {500, 300, 100, 199};
int FIELD_GOTO[] = {0, 300, 500, 200};
int BUTTON_PID[] = {500, 500, 100, 199};
int FIELD_PID[] = {0, 500, 500, 200};
color buttonColor;
color buttonActiveColor;
color fieldColor;
color textColor;
color lineColor;

String incomingTextBuffer = "";
String outgoingTextBuffer = "";

void setup() {
  currentState = "Stop";
  buttonColor = color(128);
  buttonActiveColor = color(200);
  textColor = color(255);
  fieldColor = color(50);
  lineColor = color(100);
  textSize(15);
  textAlign(CENTER, CENTER);
 
  size(600, 700);
  frameRate(25);
  oscP5 = new OscP5(this,INCOMING_PORT);
  
  step400Address = new NetAddress(DRIVER_IP,OUTGOING_PORT);
}

void draw() {
  background(0);
  //noStroke();

  
  showButton(BUTTON_SETIP, "SetIP\n" + DRIVER_IP, checkButtonArea(BUTTON_SETIP));
  showButton(BUTTON_INIT, "Init", checkButtonArea(BUTTON_INIT));
  showButton(BUTTON_STOP, "Stop", checkButtonArea(BUTTON_STOP));
  
  showButtonField(BUTTON_RUN, FIELD_RUN, currentValRun, currentState, "Run", checkButtonArea(BUTTON_RUN));
  showButtonField(BUTTON_GOTO, FIELD_GOTO, currentValGoTo, currentState, "GoTo", checkButtonArea(BUTTON_GOTO));
  showButtonField(BUTTON_PID, FIELD_PID, currentValPid, currentState, "Pid", checkButtonArea(BUTTON_PID));
  
  fill(150);
  textSize(10);
  textAlign(LEFT, TOP);
  text(outgoingTextBuffer, FIELD_RUN[0], FIELD_RUN[1], FIELD_RUN[2], width - FIELD_RUN[0]);
  text(incomingTextBuffer, FIELD_RUN[2]/2, FIELD_RUN[1], FIELD_RUN[2], width - FIELD_RUN[0]);
  textSize(15);
  textAlign(CENTER, CENTER);

  stroke(lineColor);
  line(10, FIELD_RUN[1], FIELD_RUN[2] - 10, FIELD_RUN[1]);
  line(10, FIELD_GOTO[1], FIELD_GOTO[2] - 10, FIELD_GOTO[1]);
  line(10, FIELD_PID[1], FIELD_PID[2] - 10, FIELD_PID[1]);
  if(currentState.equals("Pid")) sendPidPosition(mouseX);
}

void mousePressed()
{
  if(checkButtonArea(BUTTON_STOP))
  {
    incomingTextBuffer = "";
    outgoingTextBuffer = "";

    sendHardStop(255);
    disablePidMode(MOTOR_NO);
    currentState = "Stop";
  }
  if(checkButtonArea(BUTTON_SETIP))
  {
    sendSetDestIp();
    currentState = "Stop";
  }
  else if(checkButtonArea(BUTTON_INIT))
  {
    
    disablePidMode(MOTOR_NO);
    sendSetKval(MOTOR_NO, K_HOLD, K_RUN, K_ACC, K_DEC);
    sendRun(MOTOR_NO, 0.0);
    sendHardHiZ(255);
    currentState = "Stop";
  }
  else if (checkButtonArea(BUTTON_RUN))
  {
    incomingTextBuffer = "";
    outgoingTextBuffer = "";
    disablePidMode(MOTOR_NO);
    currentState = "Run";
  }
  /* Check run condition */
  else if (checkButtonArea(FIELD_RUN) && currentState.equals("Run"))
  {
    currentValRun = (float)(mouseX - FIELD_RUN[2]/2) / (FIELD_RUN[2]/2) * RUN_MAX;
    sendRun(MOTOR_NO, currentValRun);
  }
  else if (checkButtonArea(BUTTON_GOTO))
  {
    incomingTextBuffer = "";
    outgoingTextBuffer = "";
    disablePidMode(MOTOR_NO);
    currentState = "GoTo";
  }
  /* Check GoTo condition */
  else if (checkButtonArea(FIELD_GOTO) && currentState.equals("GoTo"))
  {
    currentValGoTo = (float)(mouseX - FIELD_GOTO[2]/2) / (FIELD_GOTO[2]/2) * GOTO_MAX;
    sendGoTo(MOTOR_NO, (int)currentValGoTo);
  }
  else if (checkButtonArea(BUTTON_PID))
  {
    incomingTextBuffer = "";
    outgoingTextBuffer = "";
    enablePidMode(MOTOR_NO);
    currentState = "Pid";
  }
  /* Check Pid condition */
  else if (checkButtonArea(FIELD_PID) && currentState.equals("Pid"))
  {
    sendPidPosition(mouseX);
  }
}

void sendPidPosition(float pos)
{
  currentValPid = (float)(pos - FIELD_PID[2]/2) / (FIELD_PID[2]/2) * GOTO_MAX;
  sendSetTargetPosition(MOTOR_NO, (int)currentValPid);
}


void sendSetDestIp() {
  String oscMsg = "/setDestIp";
  OscMessage myMessage = new OscMessage(oscMsg);
  myMessage.add(0);
  outgoingTextBuffer += oscMsg + "\n";
  println(outgoingTextBuffer);
  oscP5.send(myMessage, step400Address); 
}

/****************** Run & GoTo mode ***********************************/
void sendSetKval(int motId, int holdKval, int runKval, int accKval, int decKval) {
  String oscMsg = "/setKval";
  OscMessage myMessage = new OscMessage(oscMsg);
  outgoingTextBuffer += oscMsg;
  myMessage.add(motId);
  outgoingTextBuffer += " " + motId + " ";
  myMessage.add(holdKval);
  outgoingTextBuffer += holdKval + " ";
  myMessage.add(runKval);
  outgoingTextBuffer += runKval + " ";
  myMessage.add(accKval);
  outgoingTextBuffer += accKval + " ";
  myMessage.add(decKval);
  outgoingTextBuffer += decKval + "\n";
  oscP5.send(myMessage, step400Address); 
}

void sendRun(int motId, float speed) {
  String oscMsg = "/run";
  OscMessage myMessage = new OscMessage(oscMsg);
  outgoingTextBuffer += oscMsg + " ";
  myMessage.add(motId);
  outgoingTextBuffer += motId + " ";
  myMessage.add(speed);
  outgoingTextBuffer += speed + "\n";
  oscP5.send(myMessage, step400Address); 
}

void sendGoTo(int motId, int position) {
  String oscMsg = "/goTo";
  OscMessage myMessage = new OscMessage(oscMsg);
  outgoingTextBuffer += oscMsg + " ";
  myMessage.add(motId);
  outgoingTextBuffer += motId + " ";
  myMessage.add(position);
  outgoingTextBuffer += position + "\n";
  oscP5.send(myMessage, step400Address); 
}

void sendHardHiZ(int motId)
{
  String oscMsg = "/hardHiZ";
  OscMessage myMessage = new OscMessage(oscMsg);
  outgoingTextBuffer += oscMsg + " ";
  myMessage.add(motId);
  outgoingTextBuffer += motId + "\n";
  oscP5.send(myMessage, step400Address); 
}

void sendHardStop(int motId)
{
  String oscMsg = "/hardStop";
  OscMessage myMessage = new OscMessage(oscMsg);
  outgoingTextBuffer += oscMsg + " ";
  myMessage.add(motId);
  outgoingTextBuffer += motId + "\n";
  oscP5.send(myMessage, step400Address); 
}


/****************** PID servo mode ***********************************/
void enablePidMode(int motId)
{
  // Consult these pages for adjusting parameters
  // Japanese: https://ponoor.com/docs/step400/functional-description/servo-mode/
  // English: https://ponoor.com/en/docs/step400/functional-description/servo-mode/
  sendSetServoParam(motId, SERVO_KP, SERVO_KI, SERVO_KD);
  sendEnableServoMode(motId, true);
}

void disablePidMode(int motId)
{
  sendEnableServoMode(motId, false);
}

void sendEnableServoMode(int motId, boolean active)
{
  String oscMsg = "/enableServoMode";
  OscMessage myMessage = new OscMessage(oscMsg);
  outgoingTextBuffer += oscMsg + " ";  
  myMessage.add(motId);
  if (active){
    myMessage.add(1);
    outgoingTextBuffer += motId + " 1\n";
  }
  else {
    myMessage.add(0);
    outgoingTextBuffer += motId + " 0\n";
  }
  oscP5.send(myMessage, step400Address); 
}

void sendSetServoParam(int motId, float kP, float kI, float kD)
{
  String oscMsg = "/setServoParam";
  OscMessage myMessage = new OscMessage(oscMsg);
  outgoingTextBuffer += oscMsg + " ";  
  myMessage.add(motId);
  outgoingTextBuffer += motId + " ";
  myMessage.add(kP);
  outgoingTextBuffer += kP + " ";
  myMessage.add(kI);
  outgoingTextBuffer += kI + " ";
  myMessage.add(kD);
  outgoingTextBuffer += kD + "\n";
  oscP5.send(myMessage, step400Address); 
}

// Set PID servo mode position
void sendSetTargetPosition(int motId, int position)
{
  String oscMsg = "/setTargetPosition";
  OscMessage myMessage = new OscMessage(oscMsg);
  outgoingTextBuffer = oscMsg + " ";  
  myMessage.add(motId);
  outgoingTextBuffer += motId + " ";
  myMessage.add(position);
  outgoingTextBuffer += position + "\n";
  oscP5.send(myMessage, step400Address); 
}

/****************** UI ***********************************/
void showButtonField(int[] btn, int[] fld, float currentVal, String state, String name, boolean isMouseOver)
{
  noStroke();
  boolean isActive = false;
  if(state.equals(name)) isActive = true;
  if(isActive || isMouseOver) fill(buttonActiveColor);
  else fill(buttonColor);
  rect(btn[0], btn[1], btn[2], btn[3]);
  if(isActive)
  {
    fill(fieldColor);
    float barWidth = 0;
    if(name.equals("Run")) barWidth =  fld[2] / 2 * currentVal/RUN_MAX;
    else barWidth =  fld[2] / 2 * currentVal/GOTO_MAX;
    rect(fld[0] + fld[2] / 2, fld[1], barWidth, fld[3]);
  }
  fill(textColor);
  text(name, btn[0] + btn[2]/2, btn[1] + btn[3]/2);
  if(isActive)
  {
    if (name.equals("Run")) text(currentVal, fld[0] + fld[2]/2, fld[1] + fld[3]/2);
    else text((int)currentVal, fld[0] + fld[2]/2, fld[1] + fld[3]/2);
  }
}

void showButton(int[] bt, String nm, boolean isMouseOver)
{
  noStroke();
  if (isMouseOver) fill(buttonActiveColor);
  else fill(buttonColor);
  rect(bt[0], bt[1], bt[2], bt[3]);
  fill(textColor);
  text(nm, bt[0] + bt[2]/2, bt[1] + bt[3]/2);
}

boolean checkButtonArea(int[] btn)
{  
  if(mouseX > btn[0] && mouseX < btn[0] + btn[2] &&
    mouseY > btn[1] && mouseY < btn[1] + btn[3])
    {
      return true;
    }
  return false;
}

/* incoming osc message are forwarded to the oscEvent method. */
void oscEvent(OscMessage theOscMessage) {
  /* print the address pattern and the typetag of the received OscMessage */
  print("### received an osc message.");
  print(" addrpattern: "+theOscMessage.addrPattern());
  incomingTextBuffer += theOscMessage.addrPattern();
  String typeTag = theOscMessage.typetag();
  print(" typetag; " +typeTag);
  for(int i = 0; i < typeTag.length(); i++)
  {
    if(typeTag.charAt(i) == 'i')
    {
      print(" i ");
      print(theOscMessage.get(i).intValue());
      incomingTextBuffer += theOscMessage.get(i).intValue();
    }
    if(typeTag.charAt(i) == 'f')
    {
      print(" f ");
      print(theOscMessage.get(i).floatValue());
      incomingTextBuffer += theOscMessage.get(i).floatValue();
    }
    if(typeTag.charAt(i) == 's')
    {
      print(" s ");
      print(theOscMessage.get(i).stringValue());
      incomingTextBuffer += theOscMessage.get(i).stringValue();
    }
  }
  println("");
}
