/*---------------------------------------------------------------------------------
	$Id$
	
	bin2s: convert a binary file to a gcc asm module
	for gfx/foo.bin it'll write foo_bin (an array of char)
	foo_bin_end, and foo_bin_len (an unsigned int)
	for 4bit.chr it'll write _4bit_chr, _4bit_chr_end, and
	_4bit_chr_len


	Copyright 2003 - 2005 Damian Yerrick

	Permission is hereby granted, free of charge, to any person obtaining
	a copy of this software and associated documentation files (the
	"Software"), to deal in the Software without restriction, including
	without limitation the rights to use, copy, modify, merge, publish,
	distribute, sublicense, and/or sell copies of the Software, and to
	permit persons to whom the Software is furnished to do so, subject to
	the following conditions:

	The above copyright notice and this permission notice shall be
	included in all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
	BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
	AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
	OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
	IN THE SOFTWARE.

	$Log$
	Revision 1.3  2005/08/03 05:14:53  wntrmute
	corrected typo _size, not _len

	Revision 1.2  2005/08/01 03:42:53  wntrmute
	strip path from input file
	
	Revision 1.1  2005/07/21 13:03:05  wntrmute
	renamed labels to conform with previous bin2o
	added user configurable alignment
	

---------------------------------------------------------------------------------*/



/*
.align
.global SomeLabel_len
.int 1234
.global SomeLabel
.byte blah,blah,blah,blah...
*/

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/*---------------------------------------------------------------------------------
   Print the closest valid C identifier to a given word.
---------------------------------------------------------------------------------*/
void strnident(FILE *fout, const char *src) {
//---------------------------------------------------------------------------------
	char got_first = 0;

	while(*src != 0) {

		int s = *src++;

		/* initial digit  */
		if(isdigit(s) && !got_first)
		fputc('_', fout);  /* stick a '_' before an initial digit */

		/* convert only out-of-range characters */
		if(!isalpha(s) && !isdigit(s) && (s != '_')) {
			if(s == '-' || s == '.' || s == '/') s = '_';
		else
			s = 0;
		}

		if(s) {
			fputc(s, fout);
			got_first = 1;
		}
	}
}


//---------------------------------------------------------------------------------
int main(int argc, char **argv) {
//---------------------------------------------------------------------------------
	FILE *fin;
	size_t filelen;
	int linelen;
	int arg;
	int alignment = 4;
	if(argc < 2) {
		fputs(	"bin2s - convert binary files to assembly language\n"
						"typical usage: bin2s foo.bin bar.bin baz.bin > foo.s\n", stderr);
		return 1;
	}

  for(arg = 1; arg < argc; arg++) {

		if (argv[arg][0] == '-')
		{
			switch (argv[arg][1])
			{
				case 'a':
					
					alignment = (argc > arg) ? strtoul(argv[++arg], 0, 0) : 0;

					if ( alignment == 0 ) alignment =4;
					break;
			}
			continue;
		}
		

  	fin = fopen(argv[arg], "rb");

  	if(!fin) {
  		fputs("bin2s: could not open ", stderr);
  		perror(argv[arg]);
  		return 1;
  	}

		fseek(fin, 0, SEEK_END);
		filelen = ftell(fin);
		rewind(fin);

		if(filelen == 0) {
			fclose(fin);
			fprintf(stderr, "bin2s: warning: skipping empty file %s\n", argv[arg]);
			continue;
		}

		char *ptr = argv[arg];
		char chr;
		char *filename = NULL;
		
		while ( (chr=*ptr) ) {

			if ( chr == '\\' || chr == '/') {
				
				filename = ptr;
			}

			ptr++;
		}
		
		if ( NULL != filename ) { 
			filename++;
		} else {
			filename = argv[arg];
		}
		
		/*---------------------------------------------------------------------------------
			Generate the prolog for each included file.  It has two purposes:
			
			1. provide length info, and
			2. align to user defined boundary, default is 32bit 

		---------------------------------------------------------------------------------*/
		fprintf(	stdout, "/* Generated by BIN2S - please don't edit directly */\n"
						"\t.section .rodata\n"
						"\t.balign %d\n"
						"\t.global ", alignment);
		strnident(stdout, filename);
		fputs("_size\n", stdout);
		strnident(stdout, filename);
		printf("_size: .int %lu\n\t.global ", (unsigned long)filelen);
		strnident(stdout, filename);
		fputs("\n", stdout);
		strnident(stdout, filename);
		fputs(":\n\t.byte ", stdout);

		linelen = 0;

		while(filelen > 0) {
			unsigned char c = fgetc(fin);
			
			printf("%3u", (unsigned int)c);
			filelen--;
			
			/* don't put a comma after the last item */
			if(filelen) {

				/* break after every 16th number */
				if(++linelen >= 16) {
					linelen = 0;
					fputs("\n\t.byte ", stdout);
				} else {
					fputc(',', stdout);
				}
			}
		}

		fputs("\n\n\t.global ", stdout);
		strnident(stdout, filename);
		fputs("_end\n", stdout);
		strnident(stdout, filename);
		fputs("_end:\n\n", stdout);


		fclose(fin);
	}

	return 0;
}

