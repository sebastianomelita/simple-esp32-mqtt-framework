//#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <MQTT.h>
#include <Ticker.h>

#define WIFIRECONNECTIME  2000
#define MQTTRECONNECTIME  2000
#define LEDSOGGIORNO1 13
#define LEDSOGGIORNO2 12
#define MAXT1 		10000
#define MAXT2 		10000
#define NLEVEL1		9
#define NLEVEL2		100
#define NLEVEL3		9
#define NLEVEL5		100
#define MAXLEN		20
#define STATEPERIOD 60000

Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;

/************** DEFINIZIONE USCITE **************************/
enum outs // indice uscite
{
	OUT1			=0, // btn1, btn2
	OUT2			=1, // btn3, btn4
	OUT3			=2, // toggle1
	OUT4			=3, // slider1
	OUT5			=4, // toggle2
	OUT6			=5, // slider2
	NOUT			=6
};
/********* FINE DEFINIZIONE USCITE **************************/
unsigned long maxt[NOUT] = {MAXT1, MAXT2};
unsigned nstep[NOUT] = {NLEVEL1, NLEVEL2, NLEVEL3};
Ticker sweepTimer[NOUT];
int outval[NOUT];
int countr[NOUT];
long t[NOUT];
unsigned long tstep[NOUT];
long target[NOUT];
long targetbis[NOUT];
long midtarget[NOUT];
int direct[NOUT];
int stop[NOUT];

/************** DEFINIZIONE STATI **************************/
enum btnstates // stato pulsanti 
{
	STBTN1			=0, 
	STBTN2			=1,
	STBTN3			=2,
	STBTN4			=3,
	STBTN5			=4,
	STBTN6			=5,
	STTGL1			=6,
	STTGL2			=7,
	NSTATES         =8
};
/************** DEFINIZIONE SEGNALI **************************/
enum signals // segnali tra timer callbacks e loop() (flags)
{
	SGNBTN1			=0, 
	SGNBTN2			=1,
	SGNBTN3			=2,
	SGNBTN4			=3,
	SGNBTN5			=4,
	SGNBTN6			=5,
	SGNTGL1			=6,
	SGNTGL2			=7, 
	SGNSLD1 		=8,
	SGNSLD2 		=9,
	SGNBTNRST1		=10,
	SGNBTNRST2		=11, // non lasciare buchi nella numerazione (è correlato agli indici OUTx)
	SGNBTNRST3		=12,
	SGNBTNRST4		=13,
	SGNBTNRST5		=14,
	SGNINIT			=15,
	SGNOP			=16,
	NSGN        	=17
};
/********* FINE DEFINIZIONE SEGNALI **************************/

uint8_t stato[NSTATES];
bool signal[NSGN];
const char ssid[] = "sssiddd";
const char pass[] = "xxxxyyyyxxxx";
const char mqttserver[] = "broker";
const int mqttport = 1883;
const char intopic[] = "casa/in"; 
const char outtopic[] = "casa/out"; 
const char mqttid[] = "casa-gruppo01"; 

//WiFiClientSecure net;
WiFiClient wifi;
MQTTClient mqttClient(1024);

unsigned long lastMillis = 0;
byte count = 0;

//char* out[2]={"\"on\"", "\"off\""};
//////// 	GESTIONE DELLE FUNZIONI DEI COMANDI REMOTI     ///////////////////////////////////////////////////////////////////
// json command parser: ricerca un certo commando all’interno di una stringa e ne restituisce il valore
bool cmdParser(String &outstr, String instr, String attributo, unsigned maxlen){
	int start, ends=0;
		
	start = instr.indexOf("\""+attributo+"\":\"");

	if(start >= 0){
		start += (attributo).length() + 4;
		//scorri l'input finchè non trovi il tag di fine oppure il fine stringa
		for(ends=start+1; (ends < start + maxlen) && instr.charAt(ends)!='"' && ends < instr.length(); ends++);
		outstr = instr.substring(start, ends);
	}
	return (start >= 0);
}

// gestore comandi singolo pulsante toggle, calcola lo stato del pulsante toggle
void remoteToggle(uint8_t targetval, uint8_t sttgl, uint8_t sgntgl, uint8_t n){
	if(targetval == 255){
		signal[sgntgl] = true;
		stop[n] = true;   // normalmente è alternativo allo scivolamento per cui lo blocca
		stato[sttgl] = !stato[sttgl];		
		//t[n] = stato[sttgl]* last_t[n];
		//t[n] = stato[sttgl]* maxt[n];
	}
}

// gestore comandi singolo slider (logica senza stato)
void remoteSlider(uint8_t targetval, uint8_t sgnsld, uint8_t n){
	stop[n] = false;
	target[n] = targetval;
	targetbis[n] = targetval;
	signal[sgnsld] = true;
}

// gestore comandi coppia di pulsanti on/off o up/down (gestisce anche un'eventuale slider associato alla coppia)
// calcola lo stato dei due pulsanti 
void remoteCntrl(uint8_t targetval, uint8_t stbtna, uint8_t stbtnb, uint8_t sgnbtna, uint8_t sgnsld, uint8_t n, uint8_t mult=1){
	
	if(targetval == 255){
		targetval = 100;
		signal[sgnbtna] = true;
		if(stop[n]){
			stop[n] = false;
			target[n] = targetval*mult;
			targetbis[n] = targetval;
			stato[stbtna] = 255;
		}else{
			stop[n] = true;
			stato[stbtna] = LOW;
			stato[stbtnb] = LOW;
		}
	}else{
		if(stop[n] == false){
			stop[n] = true;
			stato[stbtna] = LOW;
			stato[stbtnb] = LOW;
		}else{
			stop[n] = false;
			target[n] = targetval;
			targetbis[n] = targetval;
			signal[sgnsld] = true;
		}
	}
}

void sweep(uint8_t n) {
	float half = tstep[n] / 2;
	if(stop[n]==false && (direct[n]>0 && t[n] < (target[n]-half) || direct[n]<0 && t[n] >= (target[n]+half))){
		t[n]=t[n]+direct[n]*tstep[n];
		Serial.println("t:"+t[n]);
		Serial.print("target+:");
		Serial.println(target[n]+half);
		Serial.print("target-:");
		Serial.println(target[n]-half);
		outval[n] = (float) t[n]/maxt[n]*100;
		countr[n] = (float) t[n]/tstep[n];
		sweepAction(outval,countr,n);
		Serial.println("++++++++++++++");
		Serial.println((String) "t:"+t[n]+" pr.value:"+outval[n]+" stop: "+String(stop[n])+" dir:"+direct[n]+" target:"+target[n]+" tmax:"+maxt[n]+" tstep:"+tstep[n]+" n:"+n+" countr:"+countr[n]);
	}else{
		sweepTimer[n].detach();
		direct[n]=0;
		stop[n]=true;
		signal[SGNBTNRST1+n] = true;
	}
}
// il target si fornisce in percentuale intera di 100 (ad es. 80)
void startSweep(unsigned nsteps,unsigned delay,unsigned long tmax,unsigned short n) {
	nstep[n]=nsteps;
	maxt[n]=tmax;
	tstep[n] = (float) tmax/nstep[n]; //durata di uno step
	if(tstep[n] > 0){
		target[n] = (float) target[n]/100*tmax;	
		if(!stop[n]){
			if(target[n] >= t[n]){
				direct[n] = 1;
			}else{
				direct[n] = -1;
			}
			if(t[n]<=0)t[n]=1;
			if(t[n]>0){
				//tstep[n]=tmax/nstep[n]; //durata di uno step
				Serial.println((String) "tstep0: "+tstep[n]+" stop0: "+String(stop[n])+" target0: "+String(target[n])+" maxt0: "+maxt[n]+" dir0: "+direct[n]+" tnow0: "+t[n]+" nstep0: "+nstep[n]+" n0: "+n);
				sweepTimer[n].detach();
				//sweepAction(n);
				sweepTimer[n].attach_ms<uint8_t>(tstep[n], sweep, n);
			};
		}
	}else{
		t[n] = target[n];
		if(target[n] > 0){
			direct[n] = 1;
		}else{
			direct[n] = -1;
		}
		//direct[n]=0;
		Serial.println("Special sweep");
		Serial.println(direct[n]);
		outval[n] = target[n];
		countr[n] = (float) (target[n] + 1) / 100 * nstep[n];
		sweepAction(outval,countr,n);
		stop[n]=true;
	}
};
// codifica stop/start e direzione della barra per la SPA
int webdir(uint8_t n) {
	int val=0;
	if(!stop[n] || tstep[n] == 0){
		val = direct[n];
	}
	return val;
}
void remoteCntrlInit() {
	for(int i=0; i<NSGN; i++){
	  signal[i] = false;
	}
	for(int i=0; i<NSTATES; i++){
	  stato[i] = 0;
	}
	for(int i=0; i<NOUT; i++){
		outval[i] = 0;
		countr[i] = 0;
		t[i] = 0;
		tstep[i] = 0;
		target[i] = 0;
		targetbis[i] = 0;
		direct[i] = 0;
		stop[i] = true; // lo scivolamento deve essere sbloccato da un tasto o da un cursore
	}
}
/////// FINE FUNZIONI DI GESTIONE DEI COMANDI REMOTI   /////////////////////////////////////////////////////////////
////   GESTIONE WIFI E MQTT    /////////////////////////////////////////////////////////////////////////////////////
void WiFiEvent(WiFiEvent_t event) {
	Serial.printf("[WiFi-event] event: %d\n", event);
	switch(event) {
	case SYSTEM_EVENT_STA_GOT_IP:
	  Serial.println("WiFi connected");
	  Serial.println("IP address: ");
	  Serial.println(WiFi.localIP());
	  connectToMqtt();
	  mqttReconnectTimer.attach_ms(MQTTRECONNECTIME, mqttConnTest);
	  break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
	  Serial.println("WiFi lost connection");
	  mqttReconnectTimer.detach(); // ensure we don"t reconnect to MQTT while reconnecting to Wi-Fi
	  wifiReconnectTimer.once_ms(WIFIRECONNECTIME, connectToWifi);
	  break;
	}
}

void mqttConnTest() {
    if (!mqttClient.connected() && WiFi.status() == WL_CONNECTED) {
		Serial.print("MQTT lastError: ");
		Serial.println(mqttClient.lastError());
		connectToMqtt();
    }
}

void connectToWifi() {
	Serial.println("Connecting to Wi-Fi...");
	WiFi.mode(WIFI_STA);
	//WiFi.disconnect();
	WiFi.begin(ssid, pass);
}

void connectToMqtt() {
	Serial.print("Connecting to MQTT...");
	Serial.print("with client id: ");
	Serial.println(mqttid);
	mqttClient.connect(mqttid, "try", "try");
	if(mqttClient.connected()){
	mqttClient.subscribe(intopic);
	}
	Serial.println("...end");
	// client.unsubscribe("/hello");
}
/*
WL_CONNECTED: assigned when connected to a WiFi network;
WL_NO_SHIELD: assigned when no WiFi shield is present;
WL_IDLE_STATUS: it is a temporary status assigned when WiFi.begin() is called and remains active until the number of attempts expires (resulting in WL_CONNECT_FAILED) or a connection is established (resulting in WL_CONNECTED);
WL_NO_SSID_AVAIL: assigned when no SSID are available;
WL_SCAN_COMPLETED: assigned when the scan networks is completed;
WL_CONNECT_FAILED: assigned when the connection fails for all the attempts;
WL_CONNECTION_LOST: assigned when the connection is lost;
WL_DISCONNECTED: assigned when disconnected from a network;
\*/
////  FINE GESTIONE WIFI E MQTT  //////////////////////////////////////////////////////////////////////////////////
void setup() {
	remoteCntrlInit();
	pinMode(LEDSOGGIORNO1, OUTPUT);
	pinMode(LEDSOGGIORNO2, OUTPUT);
	Serial.begin(115200);
	Serial2.begin(9600);
	WiFi.begin(ssid, pass);
	WiFi.onEvent(WiFiEvent);
	// MQTT brokers usually use port 8883 for secure connections.
	mqttClient.begin(mqttserver, mqttport, wifi);
	mqttClient.onMessage(messageReceived); 
	connectToWifi();
	count = 0;
	while (WiFi.status() != WL_CONNECTED && count < 10) {
		delay(500);
		count++;
		Serial.print(".");
	}
}

void loop() {
	mqttClient.loop();
	//delay(10);  // <- fixes some issues with WiFi stability
	remoteCntrlEventsParser();

	// schedulatore eventi dispositivo
	// pubblica lo stato dei pulsanti dopo un minuto
	if (millis() - lastMillis > STATEPERIOD) {
		lastMillis = millis();
		
		Serial.println("Ritrasm. periodica stato: ");
		signal[SGNINIT] = true;
	}
}
/// INIZIO CALLBACKS UTENTE  /////////////////////////////////////////////////////////////////////////////////////
/// CALCOLO USCITE DELLA FUNZIONE DI SCIVOLAMENTO (callback) 
void sweepAction(int outr[], int cr[], byte n){
	int condizout[2] = {0, 0};
	if(stop[OUT1]){
		condizout[0] = cr[OUT3];	//condizionamento digitale input 1
	}else{
		condizout[0] = stato[STTGL1]*cr[OUT1];	//condizionamento digitale input 1
	}
	if(stop[OUT2]){
		condizout[1] = cr[OUT5];	//condizionamento digitale input 2
	}else{
		condizout[1] = stato[STTGL2]*cr[OUT2];	//condizionamento digitale input 2
	}
	Serial.println((String) "r: "+cr[n]+" n:"+n);
	if(condizout[0] > 0){
		Serial2.println((String)"@l"+condizout[0]);
		Serial.println((String)"@l"+condizout[0]);
	}else{
		Serial2.println((String)"@lo");
		Serial.println((String)"@lo");
	}
};
/////// gestore messaggi MQTT in ricezione (callback)     
void messageReceived(String &topic, String &payload) {
	Serial.println("incoming: " + topic + " - " + payload);
	// Note: Do not use the client in the callback to publish, subscribe or
	// unsubscribe as it may cause deadlocks when other things arrive while
	// sending and receiving acknowledgments. Instead, change a global variable,
	// or push to a queue and handle it in the loop after calling `client.loop()`.
	
	//if(topic == intopic){
		String str;
		int start, ends=0;
		long val;
		
		// COMMANDS PARSER /////////////////////////////////////////////////////////////////////////////////////////////
		// ricerca all'interno del payload l'eventuale occorrenza di un comando presente in un set predefinito 
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		cmdParser(str,payload,"devid",MAXLEN);
		if(str == mqttid){		
		    if(cmdParser(str,payload,"to1",MAXLEN)){
				remoteToggle(255,STTGL1,SGNTGL1,OUT3);
			}
			if(cmdParser(str,payload,"to2",MAXLEN)){
				remoteToggle(255,STTGL2,SGNTGL2,OUT5);
			}
			if(cmdParser(str,payload,"on1",MAXLEN)){
				remoteCntrl(atoi(str.c_str()),STBTN1,STBTN2,SGNBTN1,SGNOP,OUT1);
			}
			if(payload.indexOf("\"off1\":\"255\"") >= 0){
				//remoteCntrl(255,STBTN2,STBTN1,SGNBTN2,SGNSLD1,OUT1,0);
				remoteCntrl(255,STBTN2,STBTN1,STBTN1,SGNOP,OUT1,0);
			}
			if(cmdParser(str,payload,"on2",MAXLEN)){
				remoteCntrl(atoi(str.c_str()),STBTN3,STBTN4,SGNBTN3,SGNOP,OUT2);
			}
			if(payload.indexOf("\"off2\":\"255\"") >= 0){
				//remoteCntrl(255,STBTN4,STBTN3,SGNBTN4,SGNSLD2,1,0);
				remoteCntrl(255,STBTN4,STBTN3,SGNBTN4,SGNOP,OUT2,0);
			}
			if(cmdParser(str,payload,"sld1",MAXLEN)){
				remoteSlider(atoi(str.c_str()),SGNSLD1,OUT4);
			}
			if(cmdParser(str,payload,"sld2",MAXLEN)){
				remoteSlider(atoi(str.c_str()),SGNSLD2,OUT6);
			}
			if(payload.indexOf("\"conf\":\"255\"") >= 0){
				signal[SGNINIT] = true;
			}
		}
	//}
}
/////    GESTORE EVENTI (callback)    /////////////////////////////////////////////////////////////////////////////////
void remoteCntrlEventsParser(){  // va dentro il loop()
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Più SPA posono comandare uno stesso dispositivo
	// lo stato dei pulsanti viene calcolato sul dispositivo ed inviato alle SPA 
	// lo stato di un pulsante viene inviato alle SPA come feedback 
	// immediatamente dopo la ricezione di un camando su quel pulsante
	// Ogni feedback certifica: l'avvenuta ricezione di un comando, lo stato del dispositivo dopo la ricezione del comando.
	// Una SPA non ricalcola in locale lo stato dei pulsanti ma si limita a copiarlo dal dispositivo.
	// Ogni SPA emula in locale la funzione di spostamento della barra (sweep) utilizzando lo stesso algoritmo di scivolamento 
	// temporale (sweepAction()) utilizzato dal dispositivo al fine di mantenere un perfetto sincronismo tra
	// disposititvo e SPA e tra le diverse SPA che comandano uno stesso dispositivo.
	// Sono previsti aggiornamenti periodici dello stato dei pulsanti inviati alle SPA su iniziativa del dispositivo per ovviare // alla eventuale perdita di un feedback a causa di un problema di connessione.
	// Le SPA possono richiedere lo stato completo dei pulsanti ad ogni connessione/riconnessione inviando il comando conf.
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// gestori dei segnali inviati da parte dei moduli di calcolo dello stato dei pulsanti
	// possono non essere raccolti tutti a seconda di quali pulsanti sono implemntati dalla SPA 
	// (potrebbe, ad es., mancare uno slider o un toggle)
	// generano i feedback dello stato dei pulsanti
	// possono essere parziali a seconda di quali pulsanti sono implemntati dalla SPA
	// se sono completi non succede nulla di anomalo in uanto la SPA potrebbe ignorare i feedback a pulsanti non implementati
	if(signal[SGNTGL1]){	
		signal[SGNTGL1] = false;
		if(stop[OUT1]){ // se up e down della scelta livello non sono attivi (no configurazione livello in corso)
			stop[OUT3] = false;
			if(stato[STTGL1]){
				target[OUT3] = targetbis[OUT3]; // 3) memorizzo il valore temporale in percentuale OK
				Serial.print("targetbis: ");
				Serial.println(targetbis[OUT3]);
				Serial.print("maxt3: ");
				Serial.println(maxt[OUT3]);
				Serial.print("maxt1: ");
				Serial.println(maxt[OUT1]);
				Serial.print("target: ");
				Serial.println(target[OUT3]);
				startSweep(NLEVEL3,0,maxt[OUT3],OUT3);
			}else{
				target[OUT3] = 0;
				Serial.print("target: ");
				Serial.println(target[OUT3]);
				startSweep(NLEVEL3,0,maxt[OUT3],OUT3);
			}
			//digitalWrite(LEDSOGGIORNO1, stato[BTN1]);
			Serial.print("TOGL1: ");
			Serial.println(stato[STTGL1]);
			mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on1\":\""+(stato[STTGL1]*targetbis[OUT3])+"\",\"off1\":\""+(stato[STTGL1]*targetbis[OUT3]*0)+"\",\"to1\":\""+stato[STTGL1]+"\",\"sp1\":\""+maxt[OUT3]+"\",\"dr1\":\""+(webdir(OUT3)*(tstep[OUT3] > 0))+"\",\"tr1\":\""+t[OUT3]+"\"}");
		}
	}
	if(signal[SGNTGL2]){		
		signal[SGNTGL2] = false;
		if(stop[OUT2]){ // se up e down della scelta livello non sono attivi (no configurazione livello in corso)
			stop[OUT5] = false;
			if(stato[STTGL2]){
				target[OUT5] = targetbis[OUT5]; // 3) memorizzo il valore temporale in percentuale OK
				Serial.print("targetbis: ");
				Serial.println(targetbis[OUT5]);
				Serial.print("maxt5: ");
				Serial.println(maxt[OUT5]);
				Serial.print("maxt2: ");
				Serial.println(maxt[OUT2]);
				Serial.print("target: ");
				Serial.println(target[OUT5]);
				startSweep(NLEVEL5,0,maxt[OUT5],OUT5);
			}else{
				target[OUT5] = 0;
				Serial.print("target: ");
				Serial.println(target[OUT5]);
				startSweep(NLEVEL5,0,maxt[OUT5],OUT5);
			}
			Serial.print("TOGL2: ");
			Serial.println(stato[STTGL2]);
			mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on2\":\""+(stato[STTGL2]*targetbis[OUT5])+"\",\"off2\":\""+(stato[STTGL2]*targetbis[OUT3]*0)+"\",\"to2\":\""+stato[STTGL2]+"\",\"sp2\":\""+maxt[OUT5]+"\",\"dr2\":\""+(webdir(OUT5)*(tstep[OUT5] > 0))+"\",\"tr2\":\""+t[OUT5]+"\"}");
		}
	}
	if(signal[SGNBTN1] || signal[SGNBTN2]){
		signal[SGNBTN1] = false;
		signal[SGNBTN2] = false;
		startSweep(NLEVEL1,0,MAXT1,OUT1);
		Serial.print("BTN1: ");
		Serial.println(stato[STBTN1]);
		Serial.println(stato[STBTN2]);
		mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on1\":\""+stato[STBTN1]+"\",\"off1\":\""+stato[STBTN2]+"\",\"sp1\":\""+maxt[OUT1]+"\",\"dr1\":\""+webdir(OUT1)+"\",\"tr1\":\""+t[OUT1]+"\"}");
	}
	if(signal[SGNBTN3] || signal[SGNBTN4]){
		signal[SGNBTN3] = false;
		signal[SGNBTN4] = false;
		startSweep(NLEVEL2,0,MAXT2,OUT2);
		Serial.print("BTN3: ");
		Serial.println(stato[STBTN3]);
		Serial.println(stato[STBTN4]);
		mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on2\":\""+stato[STBTN3]+"\",\"off2\":\""+stato[STBTN4]+"\",\"sp2\":\""+maxt[OUT2]+"\",\"dr2\":\""+webdir(OUT2)+"\",\"tr2\":\""+t[OUT2]+"\"}");
	}
	if(signal[SGNSLD1]){ 
		signal[SGNSLD1] = false;
		Serial.print("stop[OUT3]: ");
		Serial.println(stop[OUT3]);
		if(stop[OUT3]){ // se il toggle non è in scivolamento
			maxt[OUT3] = (float) target[OUT4]/100*maxt[OUT1]; // 1) target slider ---> max sweep toggle OK
			Serial.print("maxt[OUT3]: ");
			Serial.println(maxt[OUT3]);
			mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"sld1\":\""+target[OUT4]+"\"}");
		}
	}
	if(signal[SGNSLD2]){ 
		signal[SGNSLD2] = false;
		Serial.print("stop[OUT5]: ");
		Serial.println(stop[OUT5]);
		if(stop[OUT5]){ // se il toggle non è in scivolamento
			maxt[OUT5] = (float) target[OUT6]/100*maxt[OUT2]; // 1) target slider ---> max sweep toggle OK
			Serial.print("maxt[OUT5]: ");
			Serial.println(maxt[OUT5]);
			mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"sld2\":\""+target[OUT6]+"\"}");
		}
	}
	if(signal[SGNBTNRST1]){
		signal[SGNBTNRST1] = false;
		stato[STBTN1] = LOW;
		stato[STBTN2] = LOW;
		targetbis[OUT3] = (float) t[OUT1]/maxt[OUT1]*100; // 2) memorizzo il valore temporale in percentuale OK
		Serial.print("targetbis[OUT3]: ");
		Serial.println(targetbis[OUT3]);
		mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on1\":\""+stato[STBTN1]+"\",\"off1\":\""+stato[STBTN2]+"\",\"dr1\":\""+webdir(OUT1)+"\",\"tr1\":\""+t[OUT1]+"\"}");
	}
	if(signal[SGNBTNRST2]){
		signal[SGNBTNRST2] = false;
		stato[STBTN3] = LOW;
		stato[STBTN4] = LOW;
		targetbis[OUT5] = (float) t[OUT2]/maxt[OUT2]*100; // 2) memorizzo il valore temporale in percentuale OK
		Serial.print("targetbis[OUT5]: ");
		Serial.println(targetbis[OUT5]);
		mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on2\":\""+stato[STBTN3]+"\",\"off2\":\""+stato[STBTN4]+"\",\"dr2\":\""+webdir(OUT2)+"\",\"tr2\":\""+t[OUT2]+"\"}");
	}
	if(signal[SGNBTNRST3]){
		signal[SGNBTNRST3] = false;
		stato[STBTN3] = LOW;
		stato[STBTN4] = LOW;
		//stato[STTGL2] = 1; //setta il toggle
		mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on1\":\""+stato[STBTN1]+"\",\"off1\":\""+stato[STBTN2]+"\",\"dr1\":\""+webdir(OUT3)+"\",\"tr1\":\""+t[OUT3]+"\"}");
	}
	if(signal[SGNBTNRST5]){
		signal[SGNBTNRST5] = false;
		stato[STBTN3] = LOW;
		stato[STBTN4] = LOW;
		mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on2\":\""+stato[STBTN3]+"\",\"off2\":\""+stato[STBTN4]+"\",\"dr2\":\""+webdir(OUT5)+"\",\"tr2\":\""+t[OUT5]+"\"}");
	}
	if(signal[SGNINIT]){
		signal[SGNINIT] = false;
		stato[STBTN1] = LOW;
		stato[STBTN2] = LOW;
		mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on1\":\""+stato[STBTN1]+"\",\"off1\":\""+stato[STBTN2]+"\",\"sp1\":\""+maxt[OUT1]+"\",\"dr1\":\""+webdir(OUT1)+"\",\"nl1\":\""+nstep[OUT1]+"\"}");
		mqttClient.publish(outtopic, (String) "{\"devid\":\""+mqttid+"\",\"on2\":\""+stato[STBTN3]+"\",\"off2\":\""+stato[STBTN4]+"\",\"sp2\":\""+maxt[OUT2]+"\",\"dr2\":\""+webdir(OUT2)+"\",\"nl2\":\""+nstep[OUT2]+"\"}");
	}
}
////   FINE CALLBACKS UTENTE   ////////////////////////////////////////////////////////////////////////////////////////////////

