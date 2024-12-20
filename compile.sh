PROTOC="/mnt/public/szdeng/protobuf_src/protobuf/protoc"

rm -rf proto_out
mkdir proto_out
cp proto/CMakeLists.txt proto_out/

cd proto
protos=`find . -name "*.proto"`
${PROTOC} ${protos} --cpp_out=../proto_out/ --python_out=../proto_out/

if [[ ! -d ../build ]];then
  mkdir ../build
fi
cd ../build
cmake ..
make
cd -

# export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/mnt/public/szdeng/ptcgcore/build/app
