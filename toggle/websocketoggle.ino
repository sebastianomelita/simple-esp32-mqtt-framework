//#include <WiFiClientSecure.h>
#include <WiFi.h>
#include <MQTT.h>
#include <Ticker.h>

#define LED1 				13
#define LED2				12
#define LED3				11
#define LED4				10
#define WIFIRECONNECTIME  2000
#define MQTTRECONNECTIME  2000
#define MAXLEN		20
#define STATEPERIOD 60000

Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;

/************** DEFINIZIONE USCITE **************************/
enum outs // indice uscite
{
	OUT1			=0, // toggle1
	OUT2			=1, // toggle2
	OUT3			=2, // toggle3
	OUT4			=3, // toggle4
	NOUT			=4
};
/********* FINE DEFINIZIONE USCITE **************************/
//unsigned long maxt[NOUT] = {MAXT1, MAXT2};
//unsigned nstep[NOUT] = {NLEVEL1, NLEVEL2, NLEVEL3};
Ticker sweepTimer[NOUT];
int outval[NOUT];
int countr[NOUT];
long t[NOUT];
unsigned long tstep[NOUT];
long target_p[NOUT];
long target_tt[NOUT];
long targetbis[NOUT];
long midtarget[NOUT];
int direct[NOUT];
int stop[NOUT];

/************** DEFINIZIONE STATI **************************/
enum btnstates // stato pulsanti 
{
	STTGL1			=0,
	STTGL2  		=1,
	STTGL3			=2,
	STTGL4			=3,
	NSTATES         =4
};
/************** DEFINIZIONE SEGNALI **************************/
enum signals // segnali tra timer callbacks e loop() (flags)
{
	SGNTGL1			=0,
	SGNTGL2			=1, 
	SGNTGL3			=2,
	SGNTGL4			=3,
	SGNBTNRST1		=4,
	SGNINIT			=5,
	SGNOP			=6,
	NSGN        	=7
};
/********* FINE DEFINIZIONE SEGNALI **************************/

uint8_t stato[NSTATES];
bool signal[NSGN];
//&const char ssid[] = "AndroidAP1";
//const char pass[] = "pippo2503";
const char ssid[] = "xxx";
const char pass[] = "yyyy";
const char mqttserver[] = "broker.hivemq.com";
const int mqttport = 1883;
const char intopic[] = "soggiorno/in"; 
const char outtopic[] = "soggiorno/out"; 
const char mqttid[] = "soggiorno-gruppo05"; 

//WiFiClientSecure net;
WiFiClient wifi;
MQTTClient mqttClient(1024);

unsigned long lastMillis = 0;
byte count = 0;

//char* out[2]={"\"on\"", "\"off\""};
//////// 	GESTIONE DELLE FUNZIONI DEI COMANDI REMOTI     ///////////////////////////////////////////////////////////////////
// json command parser: ricerca un certo commando all�interno di una stringa e ne restituisce il valore
bool cmdParser(String &outstr, String instr, String attributo, unsigned maxlen){
	int start, ends=0;
		
	start = instr.indexOf("\""+attributo+"\":\"");

	if(start >= 0){
		start += (attributo).length() + 4;
		//scorri l'input finch� non trovi il tag di fine oppure il fine stringa
		for(ends=start+1; (ends < start + maxlen) && instr.charAt(ends)!='"' && ends < instr.length(); ends++);
		outstr = instr.substring(start, ends);
	}
	return (start >= 0);
}
// gestore comandi singolo slider (logica senza stato)
void remoteSlider(uint8_t targetval, uint8_t sgnsld, uint8_t n){
	stop[n] = false;
	target_p[n] = targetval;
	targetbis[n] = targetval;
	signal[sgnsld] = true;
}

// gestore comandi singolo pulsante toggle, calcola lo stato del pulsante toggle
void remoteToggle(uint8_t targetval, uint8_t sttgl, uint8_t sgntgl, uint8_t n){
	if(targetval == 255){
		signal[sgntgl] = true;
		stop[n] = true;   // normalmente � alternativo allo scivolamento per cui lo blocca
		stato[sttgl] = !stato[sttgl];		
	}
}
/*
// gestore comandi coppia di pulsanti on/off o up/down (gestisce anche un'eventuale slider associato alla coppia)
// calcola lo stato dei due pulsanti 
void remoteCntrl(uint8_t targetval, uint8_t stbtna, uint8_t stbtnb, uint8_t sgnbtna, uint8_t sgnsld, uint8_t n, uint8_t mult=1){
	
	if(targetval == 255){
		targetval = 100;
		signal[sgnbtna] = true;
		if(stop[n]){
			stop[n] = false;
			target_p[n] = targetval*mult;
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
			target_p[n] = targetval;
			targetbis[n] = targetval;
			signal[sgnsld] = true;
		}
	}
}

void sweep(uint8_t n) {
	float half = tstep[n] / 2;
	if(stop[n]==false && (direct[n]>0 && t[n] < (target_tt[n]-half) || direct[n]<0 && t[n] >= (target_tt[n]+half))){
		t[n]=t[n]+direct[n]*tstep[n];
		Serial.println("t:"+t[n]);
		Serial.print("target_tt+:");
		Serial.println(target_tt[n]+half);
		Serial.print("target_tt-:");
		Serial.println(target_tt[n]-half);
		outval[n] = (float) t[n]/maxt[n]*100;
		countr[n] = (float) t[n]/tstep[n];
		sweepAction(outval,countr,n);
		Serial.println("++++++++++++++");
		Serial.println((String) "t:"+t[n]+" pr.value:"+outval[n]+" stop: "+String(stop[n])+" dir:"+direct[n]+" target:"+target_tt[n]+" tmax:"+maxt[n]+" tstep:"+tstep[n]+" n:"+n+" countr:"+countr[n]);
	}else{
		sweepTimer[n].detach();
		direct[n]=0;
		stop[n]=true;
		signal[SGNBTNRST1+n] = true;
	}
}
// il target_tt si fornisce in percentuale intera di 100 (ad es. 80)
void startSweep(unsigned nsteps,unsigned delay,unsigned long tmax,unsigned short n) {
	nstep[n]=nsteps;
	maxt[n]=tmax;
	tstep[n] = (float) tmax/nstep[n]; //durata di uno step
	if(tstep[n] > 0){
		target_tt[n] = (float) target_p[n]/100*tmax;	
		if(!stop[n]){
			if(target_tt[n] >= t[n]){
				direct[n] = 1;
			}else{
				direct[n] = -1;
			}
			if(t[n]<=0)t[n]=1;
			if(t[n]>0){
				//tstep[n]=tmax/nstep[n]; //durata di uno step
				Serial.println((String) "tstep0: "+tstep[n]+" stop0: "+String(stop[n])+" target_t0: "+String(target_tt[n])+" maxt0: "+maxt[n]+" dir0: "+direct[n]+" tnow0: "+t[n]+" nstep0: "+nstep[n]+" n0: "+n);
				sweepTimer[n].detach();
				//sweepAction(n);
				sweepTimer[n].attach_ms<uint8_t>(tstep[n], sweep, n);
			};
		}
	}else{
		//t[n] = target[n];
		if(target_tt[n] > 0){
			direct[n] = 1;
		}else{
			direct[n] = -1;
		}
		//direct[n]=0;
		Serial.println("Special sweep");
		Serial.println(direct[n]);
		outval[n] = target_p[n];
		countr[n] = (float) (target_p[n] + 1) / 100 * nstep[n];
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
*/
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
		target_p[i] = 0;
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
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);
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
				remoteToggle(255,STTGL1,SGNTGL1,OUT1);
			}
			if(cmdParser(str,payload,"to2",MAXLEN)){
				remoteToggle(255,STTGL2,SGNTGL2,OUT2);
			}
			if(cmdParser(str,payload,"to3",MAXLEN)){
				remoteToggle(255,STTGL3,SGNTGL3,OUT3);
			}
			if(cmdParser(str,payload,"to4",MAXLEN)){
				remoteToggle(255,STTGL4,SGNTGL4,OUT4);
			}
			if(payload.indexOf("\"conf\":\"255\"") >= 0){
				signal[SGNINIT] = true;
			}
		}
	//}
}

String getToggleFeedback(uint8_t toggleState, uint8_t n){
	String str = "{\"devid\":\""+String(mqttid)+"\",\"to"+String(n+1)+"\":\""+String(toggleState)+"\"}";
	//Serial.println("Str: " + str);
	return str;
}
/////    GESTORE EVENTI (callback)    /////////////////////////////////////////////////////////////////////////////////
void remoteCntrlEventsParser(){  // va dentro il loop()
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pi� SPA posono comandare uno stesso dispositivo
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
		digitalWrite(LED1,stato[STTGL1]);
		Serial.print("TOGL1: ");
		Serial.println(stato[STTGL1]);
		mqttClient.publish(outtopic, getToggleFeedback(stato[STTGL1], OUT1));
	}
	if(signal[SGNTGL2]){		
		signal[SGNTGL2] = false;
		digitalWrite(LED1,stato[STTGL2]);
		Serial.print("TOGL1: ");
		Serial.println(stato[STTGL2]);
		mqttClient.publish(outtopic, getToggleFeedback(stato[STTGL2], OUT2));
	}
	if(signal[SGNTGL3]){	
		signal[SGNTGL3] = false;
		digitalWrite(LED3,stato[STTGL3]);
		Serial.print("TOGL3: ");
		Serial.println(stato[STTGL3]);
		mqttClient.publish(outtopic, getToggleFeedback(stato[STTGL3], OUT3));
	}
	if(signal[SGNTGL4]){		
		signal[SGNTGL4] = false;
		digitalWrite(LED4,stato[STTGL4]);
		Serial.print("TOGL4: ");
		Serial.println(stato[STTGL4]);
		mqttClient.publish(outtopic, getToggleFeedback(stato[STTGL4], OUT4));
	}
	if(signal[SGNINIT]){
		signal[SGNINIT] = false;
		mqttClient.publish(outtopic, getToggleFeedback(stato[STTGL1], OUT1));
		mqttClient.publish(outtopic, getToggleFeedback(stato[STTGL2], OUT2));
		mqttClient.publish(outtopic, getToggleFeedback(stato[STTGL3], OUT3));
		mqttClient.publish(outtopic, getToggleFeedback(stato[STTGL4], OUT4));
	}
}
////   FINE CALLBACKS UTENTE   ////////////////////////////////////////////////////////////////////////////////////////////////
