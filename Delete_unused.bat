@Echo off
CLS
ECHO Delete all unused and temporary files includung files in subfolders
Pause
ECHO in progress
del .\Listings\*.map
del .\Listings\*.lst
del .\Objects\*.axf
del .\Objects\*.htm
del .\Objects\*.lnp
del .\Objects\*.dep
del .\Objects\*.crf
del .\Objects\*.d
del .\Objects\*.o
del /s *Stub.scvd
del /s *.iex
del /s *.uvgui*
ECHO Done!


