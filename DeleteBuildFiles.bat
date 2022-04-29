# titled by lgd from April20  V1.0 2020-04-08
::@echo off 

# the IAR temporary files clean
@for /d /r %%c in (Debug,settings,Release) do @if exist %%c ( rd /s /q "%%c" & echo     É¾³ýÄ¿Â¼%%c) 

@for /r    %%c in (*.tmp,*.bak ,*.dep) do del "%%c"  #.dep is a temporary file in IAR project root dir

del *.pbi /s/q 
del *.xcl /s/q
del *.browse /s/q
del *.pbd /s/q
del *.o /s/q

# the Keil temporary files clean 
del /s/q .\*.dep
del /s/q .\*.bak
del /s/q .\*.o
del /s/q .\*.d
del /s/q .\*.i
del /s/q .\*.__i
del /s/q .\*.administrator
del /s/q .\*.hhhex
del /s/q .\*.bin
#del /s/q .\*.out   we may not to clean this file for it accommodate freemaster .    
#del /s/q .\*.htm   freemaster website need this file but keil Ojbect files also contain this type, keep both   
del /s/q .\*.tra
del /s/q .\*.lnp
del /s/q .\*.plg
del /s/q .\*.lst
del /s/q .\*.uvgui.*
del /s/q .\*.tmp
del /s/q .\*.crf
del /s/q .\*.map
del /s/q .\*.cout
del /s/q .\*.pbi
del /s/q .\*.pbd
del /s/q .\*.pbd.*
del /s/q .\JLinkLog.txt
del /s/q .\*.iex
del /s/q .\*.axf
del /s/q .\*.scr
del /s/q .\*.uvguix.*
#del /s/q .\*.uvoptx.*   this file contain Keil Download and Debug Configration info.   

