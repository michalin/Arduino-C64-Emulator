# Converter for Gimp raw data files
This program converts GIMP raw data files (*.data) to Hires header files. 
Usage: 
`hires.exe datafilename arrayname`

>Example: `hires tiger.data hires0` converts tiger.data to tiger.h with array name hires0


### How to generate data files 

These steps are best done with GIMP

* Crop and scale a picture to 320x200 pixels.
* Convert to black & white: Image --> Mode --> indexed...
  * Check "Use black and white (1-bit) palette
  * Color dithering: Positioned (others work too, but this looks best)
  * Click "Convert"
* Export to .data file: File --> Export 
  * Select File Type: Raw Image Data
* Make sure the .data file has a size of 64KB

