# ProtobufViewer

## Proto Setup

1. Download Protobuf from https://github.com/protocolbuffers/protobuf/releases/tag/v3.21.12 as zip or tar gz
2. Unzip/unpack
3. Follow the steps in src/README.md to install the proto compiler

## Python Usage:

Create a virtual environment (venv) and install required packages

```bash
python3 -m venv venv
source venv/bin/activate
pip install google
pip install protobuf==3.20.3    # 3.21.12 doesn't have a whl available, this is a temporary workaround
```

Receiver (listen):

```bash
python3 ProtobufViewer/receiver.py
```

Sender (connect and send 5 messages):

```bash
python3 ProtobufViewer/sender.py --count 5
```

When testing with your own proto message, you will need to recompile message_pb2.py using:

```bash
protoc --proto_path=. --python_out=. ./message.proto # Local file path references - run this line while in the ProtobufViewer/Python folder
```
