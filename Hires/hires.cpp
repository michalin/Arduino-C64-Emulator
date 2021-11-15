/* Converter for Gimp raw data files to C64 Hir-res Bitmap
    Copyright (C) 2021  Doctor Volt

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <stdio.h>
#include <stdint.h>
#include <cstring>
#include <string>

using namespace std;

int main(int argc, const char *argv[])
{
    uint8_t *buf = (uint8_t *)calloc(1, 8000);

    if (argc != 3)
    {
        printf("This program converts GIMP raw data files (*.data) to Hires header files\r\n");
        printf("Usage: hires.exe datafilename arrayname \r\n");
        printf("Example: hires tiger.data hires0\r\n");
        return -1;
    }
    string fname_in = argv[1];
    if (fname_in.rfind(".data", string::npos) == -1)
    {
        printf("Invalid filename. Must be a *.data file\r\n");
        return -1;
    }
    FILE *in = fopen(fname_in.c_str(), "r");
    if (!in)
    {
        printf("Error: Cold not open file. Does it exist?\r\n");
        return -1;
    }
    fseek(in, 0, SEEK_END);
    if (ftell(in) != 64000)
    {
        printf("Invalid file format. Must be image raw data file with 64k\r\n");
        return -1;
    }
    fseek(in, 0, SEEK_SET);
    string fname_out = fname_in.substr(0, fname_in.rfind(".data", string::npos)) + ".h";
    FILE *out = fopen(fname_out.c_str(), "w");

    uint8_t inbuf[8000][8];

    if (!fread(inbuf, 1, 64000, in))
    {
        printf("Error reading file\r\n");
        return -1;
    }

    for (int i = 0; i < 8000; i++)
        for (int j = 0; j < 8; j++)
            if (inbuf[i][j])
                buf[i] |= 128 >> j;

    uint8_t outbuf[8000];

    for (int i = 0; i < 25; i++)
        for (int j = 0; j < 40; j++)
            for (int k = 0; k < 8; k++)
                outbuf[320 * i + 8 * j + k] = buf[320 * i + j + 40 * k];

    free(buf);
    // for (int i = 0; i < 8000; i++)
    // fprintf(out, "$%02x,", outbuf[i]);

    fprintf(out, "const uint8_t %s [8000] PROGMEM= {", argv[2]);
    for (int i = 0; i < 8000; i++)
    {
        if (i && !(i % 40))
            fprintf(out, "\n");
        fprintf(out, "0x%02X,", outbuf[i]);
    }
    fseek(out, -1, SEEK_CUR);
    fprintf(out, "};");

    fclose(in);
    if (fclose(out))
    {
        printf("Error: Could not write output file\r\n");
        return -1;
    }

    printf("Hi-res header file successfully created\r\n");
    return 0;
}