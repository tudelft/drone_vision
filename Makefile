

all:
	mkdir -p build
	cmake -H. -B./build
	make -C ./build


clean:
	make -C ./build clean

mrproper:
	rm -rf ./build


cleanspaces:
	find . -name '*.[ch]' -exec sed -i {} -e 's/[ \t]*$$//' \;
	find . -name 'Makefile*' -exec sed -i {} -e 's/[ \t]*$$//' \;
