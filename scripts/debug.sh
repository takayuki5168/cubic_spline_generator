SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_DIR/..

mkdir build
cd build
cmake ..
make

./example > log.txt
gnuplot -e "plot 'log.txt' u 1:2 w lp; pause -1"
