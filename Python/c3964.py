#!usr/bin/python3
# -*-coding:Utf-8 -*
 
# 3964r Protokoll Treiber für RS232
# Michael Thelen OKT 2015
 
from serial import Serial
from datetime import datetime
import time as t
import binascii
import threading
from stepchain import stepchain
 
#---------------------------------------------------------------------------
# Schrittkette für 3964r Protokoll
# Grundklasse ist Stepchain, elementare Schrittkette
#
# Die Prozedur 3964R ist ein asynchrones, bitserielles Übertragungsverfahren. Über die Verbindung werden
# Steuer- und Nutzinformationszeichen gesendet. Um jedes Zeichen beim Empfänger wiederzuerkennen,
# und um die fehlerfreie Übertragung zu kontrollieren, werden den gesendeten Zeichen weitere Bits voranbzw.
# nachgestellt. Die Reihenfolge der Bits auf der Leitung ist:
# SA I0 I1 I2 I3 I4 I5 I6 I7 SO
# SA = Startbit
# In = Informationsbit Nr.
# SO = Stoppbit
#
# Die Steuerzeichen für die Prozedur 3964R sind der Norm DIN 66003 für den 7-Bit-Code entnommen. Sie
# werden allerdings mit der Zeichenlänge 8 Bit übertragen (Bit I7 = 0). Am Ende jedes Datenblocks wird zur
# Datensicherung ein Prüfzeichen(BCC) gesendet.
# Das Blockprüfzeichen wird durch eine exklusiv-oder-Verknüpfung über alle Datenbytes der
# Nutzinformation, inclusive der Endekennung DLE, ETX gebildet.
# Für die Informationszeichen ist kein Code vorgeschrieben (Codetransparenz).
#
# *****************************
# Senden mit der Prozedur 3964R
# Zum Aufbau der Verbindung sendet die Prozedur 3964R das Steuerzeichen STX aus. Antwortet das
# Peripheriegerät vor Ablauf der Quittungsverzugzeit (QVZ) von 2 sec mit dem Zeichen DLE, so geht die
# Prozedur in den Sendebetrieb über. Antwortet das Peripheriegerät mit NAK, einem beliebigen anderen
# Zeichen (außer DLE) oder die Quittungsverzugszeit verstreicht ohne Reaktion, so ist der
# Verbindungsaufbau gescheitert. Nach insgesamt drei vergeblichen Versuchen bricht die Prozedur das
# Verfahren ab und meldet dem Interpreter den Fehler im Verbindungsaufbau.
#
# Gelingt der Verbindungsaufbau, so werden nun die im aktuellen Ausgabepuffer enthaltenen
# Nutzinformationszeichen mit der gewählten Übertragungsgeschwindigkeit an das Peripheriegerät
# gesendet. Das Peripheriegerät soll die ankommenden Zeichen in Ihrem zeitlichen Abstand überwachen.
# Der Abstand zwischen zwei Zeichen darf nicht mehr als die Zeichenverzugszeit (ZVZ) von 220 ms
# betragen.
#
# Jedes im Puffer vorgefundene Zeichen DLE wird als zwei Zeichen DLE gesendet. Dabei wird das Zeichen
# DLE zweimal in die Prüfsumme übernommen.
#
# Nach erfolgtem senden des Pufferinhalts fügt die Prozedur die Zeichen DLE, ETX und BCC als
# Endekennung an und wartet auf ein Quittungszeichen. Sendet das Peripheriegerät innerhalb der
# Quittungsverzugszeit QVZ das Zeichen DLE, so wurde der Datenblock fehlerfrei übernommen. Antwortet
# das Peripheriegerät mit NAK, einem beliebigen anderen Zeichen (außer DLE), einem gestörten Zeichen
# oder die Quittungsverzugszeit verstreicht ohne Reaktion, so wiederholt die Prozedur das Senden des
# Datenblocks. Nach insgesamt sechs vergeblichen Versuchen, den Datenblock zu senden, bricht die
# Prozedur das Verfahren ab und meldet dem Interpreter den Fehler im Verbindungsaufbau.
#
# Sendet das Peripheriegerät während einer laufenden Sendung das Zeichen NAK, so beendet die
# Prozedur den Block und wiederholt in der oben beschriebenen Weise.
#
# Beispiel für einen fehlerlosen Datenverkehr:
# Prozedur 3964R Peripheriegerät
# STX           ->
#               <- DLE
# 1. Zeichen    ->
#               ->
#               ->
#               ->
# n. Zeichen    ->
# DLE           ->
# ETX           ->
# BCC           ->
#               <- DLE
#
# ********************************
# Empfangen mit der Prozedur 3964R
# Im Ruhezustand, wenn kein Sendeauftrag und kein Warteauftrag des Interpreters zu bearbeiten ist, wartet
# die Prozedur auf den Verbindungsaufbau durch das Peripheriegerät. Empfängt die Prozedur ein STX und
# steht ihr ein leerer Eingabepuffer zur Verfügung, wird mit DLE geantwortet.
#
# Nachfolgende Empfangszeichen werden nun in dem Eingabepuffer abgelegt. Werden zwei aufeinander
# folgende Zeichen DLE empfangen, wird nur ein DLE in den Eingabepuffer übernommen.
# Nach jedem Empfangszeichen wird während der Zeichenverzugszeit (ZVZ) auf das nächste Zeichen
# gewartet. Verstreicht die Zeichenverzugszeit ohne Empfang, wird das Zeichen NAK an das
# Peripheriegerät gesendet und der Fehler an den Interpreter gemeldet.
#
# Mit erkennen der Zeichenfolge DLE, ETX und BCC beendet die Prozedur den Empfang und sendet DLE
# für einen fehlerfrei (oder NAK für einen fehlerhaft) empfangenen Block an das Peripheriegerät.
# Treten während des Empfangs Übertragungsfehler auf (verlorenes Zeichen, Rahmenfehler), wird der
# Empfang bis zum Verbindungsabbau weitergeführt und NAK an das Peripheriegerät gesendet. Dann wird
# eine Wiederholung des Blocks erwartet. Kann der Block auch nach insgesamt sechs Versuchen nicht
# fehlerfrei empfangen werden, oder wird die Wiederholung vom Peripheriegerät nicht innerhalb der
# Blockwartezeit von 4 sec gestartet, bricht die Prozedur 3964R den Empfang ab und meldet den Fehler an
# den Interpreter.
#
# Beispiel für einen fehlerlosen Datenverkehr:
# Prozedur 3964R       Peripheriegerät
#                 <-      STX
#   DLE           -> 
#                 <-    1. Zeichen
#                 <-
#                 <-
#                 <-
#                 <-    n. Zeichen
#                 <-      DLE
#                 <-      ETX
#                 <-      BCC
#   DLE           ->
#
# ************************
# Initialisierungskonflikt
# Antwortet ein Gerät auf den Sendewunsch (Zeichen STX) seines Peripheriegerätes innerhalb der
# Quittungsverzugszeit QVZ nicht mit der Quittung DLE oder NAK, sondern ebenfalls mit dem Zeichen STX,
# liegt ein Initialisierungskonflikt vor. Beide Geräte möchten einen vorliegenden Sendeauftrag ausführen.
# Das Gerät mit der niederen Priorität stellt seinen Sendeauftrag zurück und antwortet mit dem Zeichen
# DLE. Das Gerät mit der höheren Priorität sendet daraufhin seine Daten in der vorher beschriebenen
# Weise. Nach dem Verbindungsabbau kann das Gerät mit der niederen Priorität seinen Sendeauftrag
# ausführen.
#
# niedrige Priorität     höhere Priorität
#   STX           ->
#                 <-     STX  (Konflikt)
#   DLE           -> 
#                 <-    1. Zeichen
#                 <-
#                 <-
#                 <-
#                 <-    n. Zeichen
#                 <-      DLE
#                 <-      ETX
#                 <-      BCC
#   DLE           ->
#
#
# Klassendifinition für den 3964r Treiber
# Der Treiber bedient eine Schnittstelle, welche definiert werden muss
class Dust3964r (stepchain,Serial):
    lock = threading.Lock()
    
    sendtry     = 0       # Anzahl der Sendeversuche
    sendbuff    = b""     # Sendepuffer ist leer
    readbuff    = b""     # Empfangspuffer
    telegrammOut= []      # Sendetelegramm (Puffer)
    MODE        = True    # Mit Blockprüfzeichen
    CFG_PRIO    = True    # Treiber läuft mit hoher Priorität
    CFG_PRINT   = True    # Modus Print eingeschalet (
    ETX_EN      = False   # Sequenzer erkennt, ob ein ETX nach einem DLE gültig ist
    BCC_EN      = False   # Sequenzer erkennt, das nach DLE, ETX nun das BCC folgen muss
    RealRun     = True    # Lauf im Simulator = false, in Realität True
    RUN         = False   # Der treiber läuft
    STX         = b"\x02" # chr(0x02)
    ETX         = b"\x03" # chr(0x03)
    DLE         = b"\x10" # chr(0x10)
    NAK         = b"\x15" # chr(0x15)
    LOPRIO      = False   # niedrige Prio
    HIPRIO      = True    # hohe Priorität
    M3964       = False   # Treiber läuft als 3964 ohne BCC Blocksumme
    M3964R      = True    # Treiber läuft als 3964r mit BCC Blocksumme
 
    def __init__ (self,port=None,baudrate=9600,QVZ=2.0,ZVZ=0.22,BWZ=4.0,CWZ = 3.0,SPZ=0.5,SLP= 1.4,MAXSEND=6,MAXCONNECT=6,PRIO=HIPRIO, MODE=M3964R):
        # Initialisierung der Schrittkettenklasse
        stepchain.__init__ (self)
        # Initialisierung der SchnrittstellenKlasse
        self.RS232     = Serial (port=port,baudrate=baudrate)
        self.QVZ       = QVZ        # Quittungsverzug ist 2.0 Sekunden (Buderus Doku)
        self.ZVZ       = ZVZ        # Zeichenverzugszeit 220ms (Buderus Doku)
        self.BWZ       = BWZ        # Blockwartezeit 4.0 Sekunden (Buderus Doku)
        self.CWZ       = CWZ        # Connectwartezeit 2.0 Sekunden (Wartezeit nach versuch fehlerhafter Verbindungsaufbau
        self.SPZ       = SPZ        # SendePause zeit (nach einem erfolgreichem Senden warten bis nächstes Senden
        self.SLP       = SLP        # Schlafenszeit vor dem Absenden vom DLE (muss klener als QVZ der Gegenseite sein)
        self.MAXSEND   = MAXSEND    # Maximalanzahl Sendeversuche, danach wird das Telegramm verworfen
        self.MAXCONNECT= MAXCONNECT # Anzahl maximaler Verbindungsaufbau Versuche
        self.sendERR   = 0          # Sendefehler auf 0
        self.connectERR= 0          # Verbindungsaufbau Fehler auf 0
        self.RUN       = False      # Treiber in Stop
        self.SendAtTime= 0          # Erlaube Senden ab dem Zeitpunkt
        self.MODE      = MODE       # Treibermodus einstellen (Serienmäßig nach dem Start: 3964r mit Blocksumme
        self.CFG_PRIO  = PRIO       # Modus einstellen
        self.telegrammOut= []       # Ausgangspuffer ist leer
        self.RS232.flushOutput ()   # puffer tillen
        self.RS232.flushInput ()
        self.RS232.write (self.NAK) # auf der schnittstelle mal blind am anfang ein NAK raushauen
        
 
    # hiermit kann der Modus des Treibers umgeschaltet werden. der Aufruf kann nur nach dem INIT gemacht werden, nicht während des Laufens
    # Mögliche Mode sind: PRIO = LO   : Bei einem INIT Konflikt stellt Treiber seinen Sendewunsch zurück
    #                     PRIO = HI   : Bei einem Init Konflikt besteht der Treiber auf Bestätigung Sendebereitschaft durch die Gegenseite
    #                     MODUS= 3964 : Übertragung ohne Blockprüfkennung BCC
    #                     MODUS= 3964r: Übertragung mit Blockprüfkennung BCC
    def mode (self,PRIO,MODE):
        if not self.RUN:
            self.CFG_PRIO= PRIO
            self.MODE    = MODE
 
    # Berechnet das XOR CRC für den übergebenen buff
    # buffer sollte ein bytestring sein
    # der Rückgabewert ist ein bytestring
    def crc (self,buffer):
        bcc= 0x00
        for c in buffer:
            bcc ^= c
        return bytes([bcc])
 
    # wandelt den buffer in den auszugebenen bytestring um
    # erwartet buffer als bytestrng
    # Rückgabewert ist ebenfalls ein bytestring
    def outframe (self,buffer):
        # Ein DLE in Datenstring führt zur Verdopplung von DLE
        # DLE und ETX sind der Frame vom 3964r Protokoll
        puffer= buffer.replace (self.DLE,self.DLE+self.DLE)+ self.DLE + self.ETX
        # prüfen, welches Protokoll: 3964= ohne BCC, 3964r mit BCC
        if self.MODE:
            puffer+=self.crc (puffer)
        return puffer
 
    # Befreit das Telegramm von dem 3964r Frame, prüft die Checksumme
    # Der buffer muss ein bytestring sein
    # Rückgabe: NONE, wenn irgendein Fehler aufgetreten ist, Telegramm verstümmelt, Checksum falsch
    # Wenn alles OK, Rückgabe des Telegramms als Bytestring
    def inframe (self,buffer):
        stream= buffer
        # BCC Kontrolle nur im 3964r modus
        if self.MODE:
            try:
                bufferBCC= stream[-1:]
                stream= buffer[:-1]
            except: return None
            # testen ob der Checksumme zu dem Stream passt
            if bufferBCC!=self.crc (stream):
                return None
        try:
            # Nun prüfen, ob am Ende vom Stream DLW und ETX drinsind
            if stream[-2:]!=self.DLE+self.ETX:
                return None
        except: # Inframe buffer zu klein
            return None
        return stream [:-2].replace (self.DLE+self.DLE,self.DLE)    
    
    def sendstream (self,sendepuffer):       
        buffer= self.outframe (sendepuffer)
        self.RS232.write (buffer)
        if self.CFG_PRINT:
            print (" 10r",end="")
            for c in buffer:
                print("%3.2X"% c + "s",end="")
        # der puffer wurde über die rs232 ausgegeben
 
    # Fehler in der Kommunikation: NAK ausgeben
    # Bei einem NAK wird immer auch ein flush ausgeführt
    def errNAK (self):
        self.RS232.flushOutput ()
        self.RS232.flushInput ()
        self.RS232.write (self.NAK+self.NAK+self.NAK)
        self.setnewstep (0)
 
    # eine Verzögerungszeit für das nächste Senden wird definiert
    def SetSendDelay (self,sec):
        self.SendAtTime= t.time ()+sec
 
    # Read Success wird aufgerufen, wenn ein Telegramm erfolgreich eingelesen wurde
    # Virtuelle Routine, muss überladen werden vom child    
    def ReadSuccess (self,telegram):
        pass
 
    # WriteFail wird aufgerufen, wenn win Telegramm verworfen wurde nach 6 Sendeversuchen
    # Virtuelle Routine, muss überladen werden vom child    
    def WriteFail (self,telegram):     
        pass
 
    # Write Success wird aufgerufen, wenn ein Telegram erfolgreich versendet worden ist
    # Virtuelle Routine, muss überladen werden vom child
    def WriteSuccess (self,telegram):
        pass
 
    # Routine Prüft, ob im Sendepuffer ein Auftrag vorhanden ist
    def isJob (self):
        return self.telegrammOut!=[]
 
    # Routine fügt einen neunen Sendeauftrag in den Puffer ein
    def newJob (self,job):
        Dust3964r.lock.acquire() # Thread blockieren, der part nun hier muss atomar sein
        self.telegrammOut.append (job)
        Dust3964r.lock.release() #
        if self.CFG_PRINT:
            print ("NEUER JOB EINGEGANGEN: ",job)
 
    # Routine nimmt den ältesten Sendeauftrag aus der Liste und gibt diesen Zurück
    # existiert kein Job, wird NONE zurückgegeben
    def getJob (self):    
        if not self.isJob ():
            return None   # Kein Job in der Liste
        Dust3964r.lock.acquire() # Thread blockieren, der part nun hier muss atomar sein
        job= self.telegrammOut [0]
        self.telegrammOut= self.telegrammOut [1:]
        Dust3964r.lock.release() #
        if self.CFG_PRINT:
            print ("JOB WIRD BEARBEITET: ", job)
        return job
        
 
    # Schritt 0: Der Grundschritt:
    # Steht kein aktuelles Kommando zur Ausführung an und ist inWaiting() <>0 (Zeichen im Buffer)
    # Dann neuer Schritt = 1 (Empfang überprüfen)
    def schritt_0 (self):
        if self.newstep: # Einmaliger Durchlauf in dem Schritt
            # Initialisierung der Werte
            if (self.sendERR==self.MAXSEND) or (self.connectERR==self.MAXCONNECT):
                self.WriteFail (self.sendbuff)
                self.sendbuff=b""        #
                if self.CFG_PRINT:
                    print(t.strftime("%H:%M:%S")+"."+ "%6.6d"% datetime.now().microsecond + ": Telegramm verworfen nach " , self.MAXSEND , " Fehlversuchen")
                self.sendERR=0
                self.connectERR=0
        if (self.sendbuff==b"") and self.isJob ():
            job= self.getJob ()
            if not (job is None):
                self.sendbuff=job
        self.SEND_EN= (len(self.sendbuff)!=0) and (t.time ()>self.SendAtTime)        
        if self.RS232.inWaiting () and self.RealRun:
            # Es ist ein Zeichen im Empfangspuffer
            # An dieser Stelle kann und darf es höchstens das Zeichen STX sein
            char= self.RS232.read ()
            if char != self.STX:
                # Es war kein STX, das ist auf jedenfall mal ein Fehler also: NAK senden
                if self.CFG_PRINT:
                    print(t.strftime("%H:%M:%S")+"."+ "%6.6d"% datetime.now().microsecond + ":[RX]"+ "%3.2X"% ord (char) + "r 15s [NAK: STX-START]")                  
                self.errNAK ()
            else: # Es war ein STX
                if not self.CFG_PRIO or not self.SEND_EN: # Treiber hat niedrige PRIO oder nix zum senden
                    if self.CFG_PRINT:
                        print(t.strftime("%H:%M:%S")+"."+ "%6.6d"% datetime.now().microsecond + ":[RX] 02r",end="")
                    t.sleep (self.SLP) # Für die erlaubte Antwortzeit legt sich der Prozess schlafen    
                    self.setnewstep (4) # Verbindungsaufbau 3964r läuft nun ready to receive
                elif self.SEND_EN:
                    if self.CFG_PRINT:
                        print(t.strftime("%H:%M:%S")+"."+ "%6.6d"% datetime.now().microsecond + ":[TX] 02r 02s",end="")
                    self.RS232.flushOutput ()
                    self.RS232.write (self.STX)
                    self.setnewstep (1) # verbindungsaufbau mit Konflikt: wir wollen Senden mit Hiprio
        else: # Es gibt kein Zeichen im Empfangspuffer
            if self.SEND_EN: # wir haben was zu senden
                if self.CFG_PRINT:
                    print(t.strftime("%H:%M:%S")+"."+ "%6.6d"% datetime.now().microsecond + ":[TX] 02s",end="")
                self.RS232.flushInput ()    
                self.RS232.flushOutput ()
                self.RS232.write (self.STX)
                self.setnewstep (3) # Verbindungsaufbau von uns kommt
 
    # Schritt 1: Senden (wir haben STX gesenden und erwarten ein DLE
    # Es muss ein DLE innerhalb der quittungsverzugszeit kommen
    # Alles was nicht DLE ist grund für ein NAK (Hi prio)
    def schritt_1 (self):
        if self.stepdauer>self.QVZ:
            # Quittungsverzugszeit ist abgelaufen
            if self.CFG_PRINT:
                print (" 15s [NAK: QVZ-START]")
            self.connectERR +=1 # Verbindungsaufbaufehler um 1 erhöhen
            self.SetSendDelay (self.CWZ)
            self.errNAK ()
        elif self.RS232.inWaiting ():
            # Zeichen wurde eingelesen, es muss ein DLE sein
            c= self.RS232.read ()
            if c!= self.DLE:
                # Es war aber kein DLE
                if self.CFG_PRINT:
                    print ("%3.2X"% ord (c) + "r 15s [NAK: DLE-START]")
                self.connectERR +=1 # Verbindungsaufbaufehler um 1 erhöhen
                self.SetSendDelay (self.CWZ)
                self.errNAK ()
            else: # es war ein DLE, senden ausführen
                self.sendstream (self.sendbuff)
                self.setnewstep (2)
 
    # Schritt 2: Gesendeter Datenstream muss mit DLE vom Empfänger bestätigt werden
    # DLE muss innerhalt der QVZ kommen
    def schritt_2 (self):
        if self.stepdauer>self.QVZ:
            # Quittungsverzugszeit ist abgelaufen
            if self.CFG_PRINT:
                print (" 15s [NAK: QVZ-BCC]")
            self.sendERR +=1 # sendefehler um 1 erhöhen
            self.SetSendDelay (self.BWZ)
            self.errNAK ()
        elif self.RS232.inWaiting ():
            # Zeichen wurde eingelesen, es muss ein DLE sein
            c= self.RS232.read ()
            if c!= self.DLE:            
                # Es war aber kein DLE
                if self.CFG_PRINT:
                    print ("%3.2X"% ord (c) + "r 15s [NAK: DLE-BCC]")
                self.sendERR +=1 # Verbindungsaufbaufehler um 1 erhöhen
                self.SetSendDelay (self.BWZ)
                self.errNAK ()
            else: # es war ein DLE, Telegramm wurde erfolgreich versendet
                if self.CFG_PRINT:
                    print (" 10r [OK]")
                self.WriteSuccess (self.sendbuff) # Virtuelle Routine
                self.sendbuff=b"" # Sendepuffer löschen, das telegramm austragen
                self.SetSendDelay (self.SPZ)
                self.setnewstep (0)
 
    # Schritt 3: Verbindungsaufbau von uns angestossen, wir wollen senden
    # Von uns wurde ein STX gesendet, es darf nun als Antwort kommen:
    # STX: Der Partner will selber senden
    # DLE: Alles ok, wir senden
    def schritt_3 (self):
        if self.stepdauer>self.QVZ:
            # Quittungsverzugszeit ist abgelaufen
            if self.CFG_PRINT:
                print (" 15s [NAK: QVZ-DLE START]")
            self.sendERR +=1 # sendefehler um 1 erhöhen
            self.SetSendDelay (self.CWZ)
            self.errNAK ()
        elif self.RS232.inWaiting ():
            c= self.RS232.read ()
            if c== self.DLE:
                # Das eingelesene Zeichen ist ein DLE
                # wunderbar, alles, ok, wir können senden
                self.sendstream (self.sendbuff)
                # Nach dem Senden muss mit DLE vom empfänger bestätigt werden
                self.setnewstep (2)
            elif c== self.STX:
                # Wir bekommen als Antwort auf unser STX ebenfalls ein STX zurück
                # der Klassische Initialisierungskonflikt
                if not self.CFG_PRIO:
                    # Unser Treiber hat Low Prio also alles OK, wir müssen mit DLE antworten
                    # Es folgt nun ganz normales Empfangen
                    if self.CFG_PRINT:
                        print (" 02r",end="")
                    self.setnewstep (4)
                else: # Nu gibts ein Problem.
                    # Unserer Treiber läuft auf High Prio, und die Gegenseite setzte auch ein STX ab
                    # Die Gegenseite ist also auch im High Prio Mode
                    # das ist eine etwas unkluge Sache in dem Moment
                    # wir geben ein NAK raus und initialieren neu
                    # REV 2. wir geben trotz High Prio nach und senden ein DLE
                    #if self.CFG_PRINT:
                    #    print (" 02r",end="")
                    #self.setnewstep (4)   
                    
                    if self.CFG_PRINT:
                        print (" 02r 15s [NAK: STX-STX PRIO]")
                    self.connectERR +=1 # connectfehler um 1 erhöhen
                    self.SetSendDelay (0)
                    self.errNAK ()
            else:
                print ("%3.2X"% ord (c) + "r 15s [NAK: DLE-START]")
                self.connectERR +=1 # Verbindungsaufbaufehler um 1 erhöhen             
                self.SetSendDelay (self.CWZ)                
                self.errNAK ()
      
 
    # Schritt 4: Empfangen der Daten, Verbindungsaufbau
    # Es wird das DLE gesendet f+r_ wir sind empfangsbereit
    # Danach muss innerhalb der QVZ der Stream beginnen
    def schritt_4 (self):
        if self.newstep:
            if self.CFG_PRINT:
                print (" 10s",end="")
            self.RS232.flushOutput ()
            self.RS232.flushInput ()
            self.RS232.write (self.DLE)          
        # Nach dem DLE muss nun innerhalt der ZVZ der Datenstream beginnen
        if self.stepdauer>self.ZVZ:
            # Zeichenverzugszeit ist abgelaufen NAK fehler
            if self.CFG_PRINT:
                print (" 15s [NAK: ZVZ-START]")            
            self.errNAK ()  
        elif self.RS232.inWaiting ():
            # Zeichen innerhalb der Zeit im Puffer, alles ist gut
            self.setnewstep (5)  
 
    # Schritt 5: Empfangen Datenstream
    # der Datenstream wird empfangen, bis der Parser die Sequenz DLE ETX erkennt
    def schritt_5 (self):
        # Wenn der Schritt neu aufgerufen wird, dann STX_EN auf False setzen
        if self.newstep:
            self.STX_EN  =False
            self.BCC_EN  =False
            self.readbuff=b""
        # Abfrage der Zeichenverzugszeit
        # Zeichenverzug ist aufgetreten (NAK wird gesendet)
        # Empfangsfehler hochzählen
        if t.time ()-self.starttime > self.ZVZ:
            if self.CFG_PRINT:
                print (" 15s [NAK: ERR-ZVZ]")
            self.errNAK ()
        else:    
            #solange wie zeichen im puffer oder EndeStream nicht erkannt    
            while self.RS232.inWaiting():
                #zeichen aus dem puffer lesen
                c=self.RS232.read ()
                self.starttime=t.time () # Zeit setzen beim letzten Empfangenen Zeichen
                if self.CFG_PRINT:
                    print("%3.2X"% ord (c) + "r",end="")
                self.readbuff=self.readbuff+c    
                if self.BCC_EN:
                    rec=self.inframe (self.readbuff)
                    if rec is None:
                        # Fehler beim Zerlegen vom Inframe oder Checksum fehler
                        if self.CFG_PRINT:
                            print (" 15s [NAK: ERR-BCC]")
                        self.errNAK ()
                    else:    
                        if self.CFG_PRINT:
                            print (" 10s [DLE: OK]")
                        self.ReadSuccess (rec)                               
                        self.RS232.flushInput ()
                        self.RS232.flushOutput ()                      
                        t.sleep (self.SLP) # Für die erlaubte Quittungsverzugszeit legt sich der Prozess mal schlafen
                        self.RS232.write (self.DLE)
                    self.setnewstep (0)    
                    break
                elif (c==self.DLE):
                    self.STX_EN= not (self.STX_EN)
                elif ((c==self.ETX) and self.STX_EN):
                    self.BCC_EN= True # Endekennung gültig erkannt, nun das BCC als letztes Zeichen
                else:
                    self.STX_EN=False
                    self.BCC_EN=False
        return
 
    
    # Wird immer ausgeführt vor den Schritten
    def schritt (self):
        options = {0 : self.schritt_0,1: self.schritt_1, 2: self.schritt_2, 3: self.schritt_3, 4: self.schritt_4, 5: self.schritt_5}
        options [self.step]()