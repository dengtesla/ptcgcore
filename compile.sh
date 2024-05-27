PROTOC="/mnt/public/szdeng/protobuf_src/protobuf/protoc"

rm -rf proto_out
mkdir proto_out

cd proto
protos=`find . -name "*.proto"`
${PROTOC} ${protos} --cpp_out=../proto_out/

cd ../build
cmake ..
make
cd -
