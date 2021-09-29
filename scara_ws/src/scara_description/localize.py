#!/usr/bin/env python
import os
import sys
import fileinput

fileToSearch = "scara.urdf"
textToReplace = os.getcwd()
textToSearch = "PLACEHOLDER"

tempFile = open( fileToSearch, 'r+' )
for line in fileinput.input( fileToSearch ):
    tempFile.write( line.replace( textToSearch, textToReplace ) )	
tempFile.close()

"""
Reference
https://stackoverflow.com/questions/17140886/how-to-search-and-replace-text-in-a-file
"""



