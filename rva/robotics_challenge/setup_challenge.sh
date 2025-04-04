sudo cp robotics_challenge.zip ../rva_exchange/rva_ws/src/
sudo unzip ../rva_exchange/rva_ws/src/robotics_challenge.zip -d ../rva_exchange/rva_ws/src

cd ../epd3

sudo sh copy_package.sh
sudo sh copy_script.sh

cd ../epd4

sudo sh copy_package.sh
sudo sh copy_map.sh

cd ../epd5

sudo sh copy_package.sh
sudo sh copy_files.sh