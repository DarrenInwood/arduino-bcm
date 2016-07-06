void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  String cmd = "";
  
  cmd = "page 1";
  sendNextionCommand(cmd.c_str());
}

void loop() {
  // put your main code here, to run repeatedly:

}

//// Sends a command to the Nextion display
void sendNextionCommand(const char* cmd){
  Serial.print(cmd);
  Serial.write(0xFF);
  Serial.write(0xFF);
  Serial.write(0xFF);
}
