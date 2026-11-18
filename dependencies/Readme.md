YAML-CPP
sudo apt-get install libyaml-cpp-dev

Ipopt related(Solver)
	
	Ipopt dependencies
		
		sudo apt-get install build-essential gfortran liblapack-dev libmetis-dev libopenblas-dev
		
	Ipopt:
		cd
		
		mkdir -p  ~/CoinIpopt && cd ~/CoinIpopt
		
		wget https://raw.githubusercontent.com/coin-or/coinbrew/master/coinbrew

		chmod u+x coinbrew

		./coinbrew Ipopt
		
		1
		
		y
		
		mkdir install
		
		./coinbrew build Ipopt --prefix=install --test --no-prompt --verbosity=3
		
		dependencies에 있는 coinhsl 복사 후 ~/CoinIpopt/ThirdParty/HSL/ 에 붙혀넣기
		
		cd ~/CoinIpopt
		
		./coinbrew build Ipopt --prefix=install --test --no-prompt --verbosity=3
		
		./coinbrew install Ipopt --no-prompt
		
		cd ~/CoinIpopt/install/lib
		
		ln -s libcoinhsl.so libhsl.so
		
		gedit ~/.bashrc
		
		bashrc 제일 밑단에 복사 후 추가
		-----------------------------------------------------
		export IPOPT_DIR=~/CoinIpopt/install
		export PKG_CONFIG_PATH=${PKG_CONFIG_PATH}:${IPOPT_DIR}/lib/pkgconfig
		export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${IPOPT_DIR}/lib

		# ipopt this is required for casadi
		export PATH=${PATH}:${IPOPT_DIR}/lib

		# this may speed up ipopt 
		export OMP_NUM_THREADS=1
		----------------------------------------------------
		
		source ~/.bashrc

		
casadi related(QP Solve lib)

	casadi dependencies
	
		sudo apt install gfortran liblapack-dev pkg-config --install-recommends
		
		sudo apt install swig
	
	casadi:
		cd
		
		git clone https://github.com/casadi/casadi.git -b main casadi // casadi 미설치 일때만
		
		
		기존 casadi 설치 되어 있으면 casadi 폴더 내 build 삭제
		
		cd casadi
		
		mkdir -p build
		
		cd build
		
		cmake  -DWITH_IPOPT:BOOL=ON -DWITH_HSL:BOOL=ON -DINCLUDE_PREFIX:PATH=include -DCMAKE_PREFIX:PATH=lib/cmake/casadi -DLIB_PREFIX:PATH=lib -DBIN_PREFIX:PATH=bin ..
		
		make
		
		sudo make install
		
Ultralytics for YOLOv8

	자신의 그래픽드라이버 사양에 맞는 CUDA Toolkit 및 Pytorch 설치 -> 모르면 카메라 팀원에게 문의
	pip install ultralytics

Python yaml File

	pip install pyyaml