# invoke SourceDir generated makefile for PITA.pem4f
PITA.pem4f: .libraries,PITA.pem4f
.libraries,PITA.pem4f: package/cfg/PITA_pem4f.xdl
	$(MAKE) -f C:\TI_RTOS\Workspace\PITA/src/makefile.libs

clean::
	$(MAKE) -f C:\TI_RTOS\Workspace\PITA/src/makefile.libs clean

