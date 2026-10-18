/*
 * ESP32 - Emetteur OOK (On-Off Keying) tres faible puissance, pedagogique.
 *
 * Principe : le peripherique LEDC (PWM materiel) genere une porteuse
 * carree stable sur une broche GPIO (ici GPIO 25). On active/desactive
 * cette porteuse (enable/disable du canal LEDC) pour coder des bits :
 * porteuse presente = 1, porteuse absente = 0 (OOK classique).
 *
 * Avec un simple petit bout de fil (quelques cm) connecte a la broche
 * en guise d'antenne improvisee, le signal rayonne tres faiblement et
 * peut etre capte par un SDR (type Nooelec RTL-SDR) a courte distance
 * (table de bureau), pour etre demodule et decode cote GNU Radio.
 *
 * IMPORTANT (a lire) :
 * - Puissance extremement faible (pas d'amplificateur, pas d'antenne
 *   reelle) : portee de l'ordre de quelques dizaines de centimetres a
 *   quelques metres maximum. C'est voulu : on reste dans un usage de
 *   labo/bureau, jamais pour emettre "loin" ou "fort".
 *   N'ajoute pas d'amplificateur RF ni d'antenne accordee : ce montage
 *   est pense comme une experience de table, pas comme un emetteur.
 * - Frequence choisie (27 MHz) dans la bande CB, zone generalement
 *   toleree pour ce type d'experimentation a tres faible niveau en
 *   France, mais verifie la reglementation locale (ARCEP) si tu as un
 *   doute, et reste a tres faible duree/puissance de test.
 * - Garde les sessions de test courtes (quelques secondes/minutes),
 *   pas de fonctionnement continu 24/7.
 */

#define OOK_PIN      25          // broche GPIO portant la porteuse (+ petit fil = antenne improvisee)

// NOTE API : depuis le core ESP32 Arduino 3.x, ledcSetup()/ledcAttachPin()
// ont ete remplaces par ledcAttach(pin, freq, resolution_bits), et
// ledcWrite() prend desormais directement le PIN (plus le numero de
// canal). Ce fichier utilise la nouvelle API. Si tu es sur un core 2.x
// (l'erreur de compilation dirait alors "ledcAttach not declared" au
// lieu de "ledcSetup not declared"), remets l'ancienne forme avec
// ledcSetup(channel, freq, res) + ledcAttachPin(pin, channel) + ledcWrite(channel, duty).

// IMPORTANT : la frequence max atteignable par le LEDC depend de la
// resolution demandee (freq_max ~= APB_CLK / 2^resolution). Pour viser
// une porteuse haute (dizaines de MHz), il faut une resolution minimale
// (1 bit, donc juste du on/off, pas besoin de duty variable ici puisqu'on
// module en activant/desactivant tout le canal, pas le duty cycle).
#define LEDC_TIMER_BIT 1
#define CARRIER_FREQ_HZ 27000000UL  // cible : 27 MHz, dans la plage recue par la Nooelec

// NOTE : selon la revision du chip/SDK, le LEDC peut ne pas tenir
// exactement 27 MHz. Si ledcSetup() echoue ou que rien n'apparait cote
// GNU Radio, baisse CARRIER_FREQ_HZ par paliers (ex: 20 MHz, 13.56 MHz,
// 10 MHz) jusqu'a trouver une valeur stable sur ton module, puis ajuste
// la frequence centrale du flowgraph de reception en consequence.

#define BIT_DURATION_MS 300      // duree d'un bit OOK, volontairement longue pour laisser
                                  // le temps a l'attache/detache du LEDC de se stabiliser
                                  // avant de rebasculer (diagnostic : les bits a 0 etaient
                                  // trop courts pour etre vus par le recepteur)

const char* MESSAGE = "HELLO";

// CORRECTIF : a resolution 1 bit, ledcWrite(OOK_PIN, 0) ne coupe pas
// vraiment la porteuse sur certaines cartes/cores (confirme par la
// reception : le signal reste visible en continu dans GQRX, meme
// entre deux envois). On force donc une coupure franche en detachant
// completement le LEDC de la broche et en la forcant a l'etat bas,
// plutot que de compter sur un duty cycle a 0%.
bool carrierAttached = false;

void ledcCarrierOn() {
  if (!carrierAttached) {
    ledcAttach(OOK_PIN, CARRIER_FREQ_HZ, LEDC_TIMER_BIT);
    carrierAttached = true;
  }
  ledcWrite(OOK_PIN, 1);
}

void ledcCarrierOff() {
  if (carrierAttached) {
    ledcDetach(OOK_PIN);
    carrierAttached = false;
  }
  pinMode(OOK_PIN, OUTPUT);
  digitalWrite(OOK_PIN, LOW);
}

void sendBit(bool bitValue) {
  if (bitValue) {
    ledcCarrierOn();
  } else {
    ledcCarrierOff();
  }
  delay(BIT_DURATION_MS);
}

// Marqueur de debut de trame simple (motif facile a reperer visuellement
// sur le waterfall / dans le flux demodule) : 1010101011
void sendPreamble() {
  bool pattern[] = {1,0,1,0,1,0,1,0,1,1};
  for (bool b : pattern) sendBit(b);
}

void sendByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    sendBit((b >> i) & 0x01);
  }
}

void sendMessage(const char* msg) {
  sendPreamble();
  for (size_t i = 0; i < strlen(msg); i++) {
    sendByte((uint8_t)msg[i]);
  }
  ledcCarrierOff();  // s'assurer que ca retombe a l'etat eteint entre trames
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("ESP32 OOK TX - demarrage");

  // attachement de diagnostic au demarrage, juste pour afficher la
  // frequence reellement obtenue (ledcAttach renvoie un bool, pas la
  // frequence : on la relit avec ledcReadFreq)
  bool ok = ledcAttach(OOK_PIN, CARRIER_FREQ_HZ, LEDC_TIMER_BIT);
  if (!ok) {
    Serial.println("ERREUR : ledcAttach a echoue (frequence/resolution non supportee).");
  } else {
    double actualFreq = ledcReadFreq(OOK_PIN);
    Serial.printf("LEDC attache sur pin %d, frequence obtenue : %.0f Hz\n", OOK_PIN, actualFreq);
  }
  carrierAttached = true;
  ledcCarrierOff();  // detache proprement et force l'etat bas avant de commencer
}

void loop() {
  Serial.println("Envoi du message...");
  sendMessage(MESSAGE);
  delay(2000);  // pause entre deux trames
}
