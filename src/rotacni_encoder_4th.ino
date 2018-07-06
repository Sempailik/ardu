// Arduino rotacni encoder
//dodelat:
//udelat podminku kdzy hodnota milis bude vetsi nez XXXXXX tak se vynuluje aby nedoslo k preteceni
//predelat casovani pomoci metody SET a EXPIRED !!!

 /*
  *  Vypocet rychlosti:
  *  obvod kruhu = 2 * Pi * r (remenice ma r= 0,015m)
  *  obvod kruhu = 0,09424777m
  *  1 metr / obvod kruhu = otacky  => 1 / 0,09424777 = 10,610329
  *  otacky * pocet pulzu na otacku =>  10,610329 * 200 = 2122,06590789 pulzu na 1m
  *  draha na 1 pulz 1 / 2122.06590789 => 0,00047123889803 pulz/m
  *
  *  pulz_na_metr = 0.00047123889803
  *
  *  rychlost = (pocet pulzu * draha ) / cas
  *  rychlost = ( pocetPulzu * pulz_na_metr ) / ((uplynulyCas)/1000.0 ) ;
  *  rychlost = ( pocetPulzu * pulz_na_metr * 1000.0) / (uplynulyCas) ; //snad rychlejsi vypocet, protoze misto deleni nasobim
  *
  */
#include "interval.h"

#define version 1.01
#define pulz_na_metr 0.00047123889803
#define pulz_na_metr_per_mil 471.23889803846
#define pinKanalA 2         //pin 2
#define pinKanalB 3         //pin 4   //puvodne 3
#define pinPreruseni 0      // 0 jako INT0 coz = pin 2
#define pinPreruseniB 1      // 1 jako INT1 coz = pin 3
#define power_led 13        //pin 13
#define strobe_led 12       //pin 12
#define previjeni_led 11    //pin 11
#define mereni_led 10       //pin 10
#define motor 9             //pin 9
#define tlacitko_previjeni 8 //pin 8
//#define tlacitko_stop 7      //pin 7
#define dobaPoKtereRozsvitiLed 500 //ms
#define dobaPoKtereZhasneLed 1000 //ms
#define casMereni  240000 //cas po ktery se bude merit v milisekundach 240000 = 240s = 4min
#define periodaMereni 10  //4ms perioda mereni a odesilani na seriovy port ve vysledku je to 5ms
#define rychlost_zmeny_mereni 2.1 //rychlost po ktere se zacne merit poctem pulzu, do te doby merit  periodou mezi pulzy
#define korekce1 1.0575   //korekce do rychlosti 0.9m/s je 5.75% = 1.0575
#define korekce2 1.12   //korekce 0.25m/s do rychlosti 0.9m/s je 12.0% = 1.12
#define korekcni_hranice 0.8
#define korekcni_hranice_spodni 0.25

Interval timer, timer_strobe_Led_ON, timer_strobe_Led_OFF;

// pomocné proměnné
short int flag_interrupt = 0;
// PWM hodnota  0-255,
// 10%=25, 20%=51, 30%=76, 40%=102, 50%=127, 60%=153, 70%=178, 80%=204, 90%=229,
int velikost_pwm = 60;
int velikost_pwm_mene = 160;
int velikost_pwm_previjeni = 153;//229;
volatile short int pocetPulzu = 0;
// volatile byte pocetPulzuB = 0;
volatile byte smer = 0;
//float otacky = 0.0;
unsigned long celkovy_pocet_pulzu = 0;
long delka = 0;
float rychlost = 0;
//unsigned long soucetML = 0;
unsigned long staryCas = 0;
unsigned long staryCasMicro = 0;
unsigned long cas_zacatku_mereni = 0;
unsigned long cas_stisku_tlacitka = 0;
unsigned short stisknuto = LOW;
unsigned short povolen_stisk = 1;
char prijato;
short int synchronizace = 0;
unsigned short flag_merit = 0;
unsigned short stav_tlacitko_previjeni = 0;
unsigned short stav_tlacitko_stop = 0;
bool predchoziA = false;
bool aktualniA = false;
bool predchoziB = false;
bool aktualniB = false;
unsigned long cas_pulzu = 0;
unsigned long predchozi_cas_pulzu = 0;
unsigned long celkova_delka_pulzu = 0;
unsigned long delka_pulzu = 0;
float delkova_rychlost = 0;
float tmp_delkova_rychlost = 0;
short int my_pocetPulzu = 0;

void setup() {
  // komunikace po sériové lince rychlostí 9600 baud
  //Serial.begin(9600);
  Serial.begin(115200);
  //Serial.begin(250000);
  //Serial.begin(76800);
  //Serial.begin(38400);
  // nastavení směru vstupního pinu
  pinMode(pinKanalA, INPUT_PULLUP);
  pinMode(pinKanalB, INPUT_PULLUP);
  pinMode(tlacitko_previjeni, INPUT_PULLUP);
//  pinMode(tlacitko_stop, INPUT_PULLUP);
  pinMode(previjeni_led, OUTPUT);
  pinMode(mereni_led, OUTPUT);
  pinMode(power_led, OUTPUT);
  pinMode(strobe_led, OUTPUT);
  pinMode(motor, OUTPUT);

  analogWrite(motor, 0);
  // nastavení vstupního pinu pro využití přerušení,
  // při detekci přerušení pomocí sestupné hrany (FALLING)
  // bude zavolán podprogram prictiPulz
  //attachInterrupt(pinPreruseni, prictiPulz, FALLING);
  attachInterrupt(pinPreruseni, prictiPulz, CHANGE);
  attachInterrupt(pinPreruseniB, prictiPulz, CHANGE);

  digitalWrite(strobe_led, LOW);
  
}
void loop() {

  // místo pro další příkazy
  digitalWrite(power_led, HIGH);

  if ((prijato=='s') && (!flag_merit) )
  {
    flag_merit = 1;
    cas_zacatku_mereni = millis();
    delka = 0;
    celkovy_pocet_pulzu = 0;
    char temp[25];
    Serial.print("verze prog.:");
    Serial.println(version);
    sprintf(temp,"mereni probiha po %i ms",periodaMereni);
    Serial.println(temp);
    Serial.println("; rychlost; synchronizace; delkova_rychlost; delka;");

    timer.set(periodaMereni);
    timer_strobe_Led_ON.set(dobaPoKtereRozsvitiLed);
    timer_strobe_Led_OFF.set(dobaPoKtereZhasneLed);
  }
  else if(prijato=='p')
  {
   // digitalWrite(led, LOW);
    flag_merit = 0;

  }

//Rozsviceni SYNCHRONIZACNI LED PO DOBE
//if(timer_strobe_Led_ON.expired())
    if (flag_merit == 1 &&
      ((millis() - cas_zacatku_mereni) > dobaPoKtereRozsvitiLed) &&
      ((millis() - cas_zacatku_mereni) < dobaPoKtereZhasneLed))
    {
      digitalWrite(strobe_led, HIGH);
      synchronizace=1;
      //Serial.println("synch 1");
    }
    else
    {
      digitalWrite(strobe_led, LOW);
      synchronizace=0;
      //Serial.println("synch 0");
    }


//ZHASNUTI SYNCHRONIZACNI LED PO DOBE
  //if ((millis() - cas_zacatku_mereni) > dobaPoKtereZhasneLed)
/*
  if(timer_strobe_Led_OFF.expired())
  {
    digitalWrite(strobe_led, LOW);
    synchronizace=0;
    Serial.println("synch 0");
  }
*/

  // pokud je rozdíl posledního uloženého času a aktuálního
  //if ( (uplynulyCas > periodaMereni) && flag_merit  )
  if ( (timer.expired()) && flag_merit  )
  {
    unsigned long uplynulyCas = (millis() - staryCas);

    unsigned long tmp_celkova_delka_pulzu;

    do                              //bezpecne predani hodnot z pulzru, tak aby nedoslo zrovna ke zmene
    {                               //hodnoty, pri predavani nebo vypoctu a neni potreba zakazovat preruseni
      flag_interrupt=0;             //tim padem neprijdu o data z pulzru

      my_pocetPulzu = pocetPulzu;

      celkovy_pocet_pulzu += pocetPulzu;
      tmp_delkova_rychlost = delkova_rychlost;
      tmp_celkova_delka_pulzu = celkova_delka_pulzu;
      celkova_delka_pulzu = 0;
      pocetPulzu = 0;
    } while (flag_interrupt==1);

      // vypnutí detekce přerušení po dobu výpočtu a tisku výsledku
    //detachInterrupt(pinPreruseni);

    // výpočet průtoku podle počtu pulzů za daný čas
    // se započtením kalibrační konstanty
    // otacky = ((1000.0 / (millis() - staryCas)) * pocetPulzu) / kalibrFaktor;
    // 1 ot/s = (50 p * 0,02) / 1 s

    //otacky = ( pocetPulzu * 0.02 ) / (((millis() - staryCas))/1000.0 ) ;

    //rychlost = ( my_pocetPulzu * pulz_na_metr ) / ((uplynulyCas)/1000.0 ) ;
    rychlost = ( my_pocetPulzu * pulz_na_metr * 1000.0) / (uplynulyCas) ; //snad rychlejsi vypocet, protoze misto deleni nasobim
    //rychlost = ( my_pocetPulzu * 3.184713375 ) / (millis() - staryCas)   ;
    if(rychlost < korekcni_hranice)
    {
      rychlost *= korekce1;
    }


    if(smer==1)
      {
        if (rychlost != 0)rychlost *= -1;
      }


/*
    char rychlost_temp[6];
    //char otacky_temp[7];
    // 4 is mininum width, 2 is precision; float value is copied onto rychlost_temp
    dtostrf(rychlost, 4, 2, rychlost_temp);
    //dtostrf(otacky, 5, 2, otacky_temp);
*/
    if(smer==1)
      {
        delka -= 47 * my_pocetPulzu; //
      }
      else
      {
        delka += 47 * my_pocetPulzu; //
      }

    //tmp_delkova_rychlost = pulz_na_metr / (delka_pulzu/1000000.0);
    //tmp_delkova_rychlost = (pulz_na_metr * 1000000.0) / (delka_pulzu);
    //tmp_delkova_rychlost = (pulz_na_metr_per_mil) / (delka_pulzu);
    float tmp_cas_pulzu = tmp_celkova_delka_pulzu / my_pocetPulzu;
    tmp_delkova_rychlost = (pulz_na_metr_per_mil) / (tmp_cas_pulzu);
/*
    if((korekcni_hranice_spodni < tmp_delkova_rychlost) && (tmp_delkova_rychlost < korekcni_hranice))
    {
      tmp_delkova_rychlost *= korekce2;
    }
*/
    if(smer==1)
    {
      if (tmp_delkova_rychlost != 0) tmp_delkova_rychlost *= -1;
    }
    /*
    if( 0 == (int)(rychlost*100) )
    {
      tmp_delkova_rychlost = 0;
    }
    */

/*
    char tmp_cpp[6];
    ltoa(celkovy_pocet_pulzu, tmp_cpp, 5);
    char tmp_delka[8];
    itoa(delka, tmp_delka, 7);
    char del_rych_tmp[7];
    dtostrf(tmp_delkova_rychlost, 4, 2, del_rych_tmp);
    char delka_tmp[7];
    float i = (double)delka/1000;
    dtostrf(i, 4, 2, delka_tmp);
*/
//    char tmp[40];
    //sprintf(tmp, "r:%s; s:%hi; d:%i; p:%i; cp:%i; c:%i;", rychlost_temp, synchronizace, delka, my_pocetPulzu, celkovy_pocet_pulzu, uplynulyCas);
    //sprintf(tmp, ";%s;%hi;%i;%i;", rychlost_temp, synchronizace, my_pocetPulzu, uplynulyCas);
//    sprintf(tmp, ";%s;%hi;%i;%i;", rychlost_temp, synchronizace, my_pocetPulzu, uplynulyCas);
    //sprintf(tmp, ";%s;%hi;%i;%i;", rychlost_temp, synchronizace, my_pocetPulzu, (micros() - staryCasMicro));
//    sprintf(tmp, ";%s;%hi;%i;%i;%i;%s;%s", rychlost_temp, synchronizace, my_pocetPulzu, uplynulyCas, celkovy_pocet_pulzu, delka_tmp, del_rych_tmp);

    Serial.println(String("") + ";" + rychlost + ";"
                                    + synchronizace +";"
                                    + tmp_delkova_rychlost +";"
                                    //+ my_pocetPulzu +";"
                                    //+ uplynulyCas +";"
                                    //+ celkovy_pocet_pulzu +";"
                                    + ((double)delka/100000) +";"

                                  );

    // uložení aktuálního času pro zahájení dalšího měření
    staryCas = millis();

    //digitalWrite(led, LOW);
    // povolení detekce přerušení pro nové měření
    //attachInterrupt(pinPreruseni, prictiPulz, FALLING);

    timer.set(periodaMereni);
  }
  else
  {
    //digitalWrite(mereni_led, LOW);
    if (Serial.available() > 0)
    {
      prijato = Serial.read();
      //Serial.println(prijato);
    }

  }

  stav_tlacitko_previjeni = digitalRead(tlacitko_previjeni);

  if(flag_merit)
  {
    digitalWrite(mereni_led, HIGH);
    analogWrite(motor, velikost_pwm);
  }
  else
  {
    if(stav_tlacitko_previjeni==LOW)  //ZMACKNUTE stav_tlacitko_previjeni==LOW
    {
      digitalWrite(previjeni_led, HIGH);
      analogWrite(motor, velikost_pwm_previjeni);
    }
    else
    {
      digitalWrite(mereni_led, LOW);
      digitalWrite(previjeni_led, LOW);
      analogWrite(motor, 0);
    }
  }

/*
  stav_tlacitko_stop = digitalRead(tlacitko_stop);
  // kontrola stisku tlačítka a stavu proměnné povolení
  if(stav_tlacitko_stop == LOW  &&  povolen_stisk == 1)
  {
    stisknuto = HIGH;
    Serial.println("STOP");
  }
  // pokud je stisknuto, do proměnné cas_stisku_tlacitka se uloží čas stisku a zakáže se další stisknutí
  if(stisknuto == HIGH )
  {
    cas_stisku_tlacitka = millis();
    povolen_stisk = 0;
    stisknuto = LOW;
  }
  // pokud je při zákazu stisku naměřen rozdíl času stisku a momentálního času větší jak 1000 ms = 1 s,
  // tak je povolen další stisk
  if(povolen_stisk == 0)
  {
    if((millis()-cas_stisku_tlacitka) > 1000 )
    {
      flag_merit = 0;
      povolen_stisk = 1;
    }
  }
*/

}
// podprogram pro obsluhu přerušení
void prictiPulz() {
  // inkrementace čítače pulzů
  flag_interrupt = 1;
  pocetPulzu++;

  aktualniA = (bool)digitalRead(pinKanalA);
  aktualniB = (bool)digitalRead(pinKanalB);

       if((aktualniA == false) && (predchoziA == true))
       {
          if((aktualniB == false) && (predchoziB == false))
          {
            smer = 0;
          }
          else if((aktualniB == true) && (predchoziB == true))
          {
            smer = 1;
          }
       }
       else if((aktualniA == false) && (predchoziA == false))
       {
          if((aktualniB == true) && (predchoziB == false))
          {
            smer = 0;
          }
          else if((aktualniB == false) && (predchoziB == true))
          {
            smer = 1;
          }
       }
       else if((aktualniA == true) && (predchoziA == false))
       {
          if((aktualniB == true) && (predchoziB == true))
          {
            smer = 0;
          }
          else if((aktualniB == false) && (predchoziB == false))
          {
            smer = 1;
          }
       }
       else if((aktualniA == true) && (predchoziA == true))
       {
          if((aktualniB == false) && (predchoziB == true))
          {
            smer = 0;
          }
          else if((aktualniB == true) && (predchoziB == false))
          {
            smer = 1;
          }
       }

  predchoziA = aktualniA;
  predchoziB = aktualniB;

  /*
  unsigned long currentMillis = millis();
  if ((unsigned long)(currentMillis - previousMillis) >= interval)
  {
    previousMillis = currentMillis;
  }
  */
  cas_pulzu = micros();
  delka_pulzu = cas_pulzu - predchozi_cas_pulzu;
  celkova_delka_pulzu += delka_pulzu;
  predchozi_cas_pulzu = cas_pulzu;

}
