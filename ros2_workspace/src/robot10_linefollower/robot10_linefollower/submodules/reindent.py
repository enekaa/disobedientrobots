#!/usr/bin/python3
''' 
reindent.py

this script is a tool for fixing inconsistent indentation when your chosen editor chooses to hose your tabs with a bunch of spaces 
'''

import os, sys, argparse
from io import StringIO

def reindent(buffer, tabs=2, tospaces=False, nospaces=True, fromtabs=None, backupsuffix="~"):
    if fromtabs == None:
        fromtabs = tabs
    outlines = []
    with StringIO(buffer) as fin:
        buf = fin.read()
        lines = buf.splitlines()
        for lineno, line in enumerate(lines):
            line = line.rstrip()
            indent = 0
            pos = 0
            while pos < len(line) and line[pos] in [' ', '\t']:
                if line[pos] == ' ':
                    indent = indent + 1
                elif line[pos] == '\t':
                    indent = (indent//fromtabs + 1) * fromtabs
                else:
                    break
                pos = pos + 1
            line = line[pos:]
            if indent - (indent//tabs)*tabs > 0:
                if nospaces and not tospaces:
                    #line = '\t' + line
                    raise RuntimeError(f"[line {lineno}] Indentation does not line up with tab-stops, aborting.")
                else:
                    line = (' '*(indent - (indent//tabs)*tabs)) + line
            for i in range(0, indent//tabs):
                if tospaces:
                    line = (' '*tabs) + line
                else:
                    line = '\t' + line
            outlines.append(line)
    return "\n".join(outlines)

def backup_file(filename, backupsuffix="~", nobackup=False, overwriteok=True):
	try:
		import os
		outputfilename = filename
		if not nobackup and os.path.exists(outputfilename):
			suff = 0
			backupfilename = f"{outputfilename}{backupsuffix}"
			while os.path.exists(backupfilename):
				suff = suff + 1
				backupfilename = f"{outputfilename}{backupsuffix}{suff}"
			#print(f"backing up existing output file {outputfilename}.py as {backupfilename}...")
			#overwriteok = False
			with open(outputfilename, "rb") as fin:
				buf = fin.read()
				with open(backupfilename, "wb") as fout:
					fout.write(buf)
					#print(f"... wrote {len(buf)} bytes to backup file.")
	except Exception as err:
		print(f"Backing up original file failed: {err}")
		return False
	return True

def reindent_file(filename, backupsuffix="~", backupcount=None, *args, **kwargs):
    import os

    with open(filename, "r") as finput:
        buffer = finput.read()

    result = reindent(buffer, *args, **kwargs)

    if not backup_file(filename, backupsuffix = backupsuffix, nobackup = backupcount==0):
        raise RuntimeError(f"Failed to backup original input file, refusing to overwrite: {filename}")

    with open(filename, "w") as fout:
        written = fout.write(result)

    return True, len(buffer), written

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description="Fix in-place python scripts with inconsistent indentation")
	parser.add_argument("-t", "--tabs", type=int, default="2", help="Output tab width in columns. Default is 2.")
	parser.add_argument("-s", "--space", action="store_true", help="Indent with spaces instead of tabs")
	parser.add_argument("-m", "--mixed", action="store_true", help="Allow indentation between tab stops by padding with spaces.")
	parser.add_argument("-f", "--from", dest="from_tabs", action="store", type=int, default=None, help="Tab size in input to convert from. Defaults to -t output tab size.")
	parser.add_argument("-b", "--backup-suffix", action="store", type=str, default="~", help="Backup filename suffix")
	parser.add_argument("-n", "--backup-count", action="store", type=str, default=-1, help="Maximum number of backups to keep. If negative, removes the oldest backup when count is reached. If positive, rotate backups and remove the oldest.")
	parser.add_argument("-c", "--cleanup", action="store_true", default=False, help="Automatically detect indented blocks in python mode and reindent each level by one block")
	parser.add_argument("filename", nargs="+", type=str, help="The file to reindent")
	opts = parser.parse_args()
	for fn in opts.filename:
		print(f"Reindenting {repr(fn)}...")
		reindent_file(fn, backupsuffix=opts.backup_suffix, backupcount=opts.backup_count, tabs=opts.tabs, tospaces=opts.space, nospaces=not opts.mixed, fromtabs=opts.from_tabs)
