#!usr/bin/python3
# -*-coding:Utf-8 -*

import time as t

# Grundklasse einer Schrittkette
class stepchain:
    step=0
    laststep=0
    nextstep=0
    newstep=True
    starttime=t.time ()
    stepdauer=0

    def __init__ (self,):
        self.step=255  # 255 = Initialisierung, bei Neustart mit Run erfolgt auf jedenfall ein newstep=true
        self.laststep=0
        self.nextstep=0
        self.newstep=True
        self.starttime=t.time ()
        self.stepdauer=0

    def schritt (self):
        pass

    def running (self):
        self.newstep=self.step!=self.nextstep # Schrittwechsel erkannt
        if self.newstep:
            self.starttime=t.time ()
            self.laststep=self.step
        self.stepdauer=t.time ()-self.starttime
        self.step=self.nextstep
        # Aufruf: wird immer vor den Schritten ausgeführt
        self.schritt()

    def setnewstep (self,step):
        self.nextstep= step

    def schrittDauer (self):
        return t.time ()-self.starttime

    def triggerDauer (self):
        self.starttime=t.time ()
