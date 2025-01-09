openai_ros
==========

ROS1 wrapper for (OpenAI Python Library)[https://pypi.org/project/openai/].

How to start program
--------------------

Start from sample lamuch file
```
$ export OPENAI_API_KEY="xxxxxxxxx"
$ roslaunch openai_ros openai.launch model:=gpt-3.5-turbo
```

or

```
$ roslaunch openai_ros openai.launch key:="xxxxxxxx" model:=gpt-3.5-turbo
```

or put following code within your launch file.
```
  <group ns="openai">
    <node pkg="openai_ros" type="openai_node.py" name="openai" output="screen">
      <rosparam command="load" file="openai_credentials.json" />
    </node>
  </group>
```
The contents of `openai_credentials.json` file is something like
```
key: xxxxxxxxx
max_tokens: 256
model: gpt-3.5-turbo
```

When you want to use Azure OpenAI, run

```
roslaunch openai_ros openai.launch endpoint:="https://xxxxxxx.azure.com" key:="<api key>" model:="<deployed chat model instance name>"  
```

When you use Azure OpenAI, firstly please prepare some chat or completion model instance deployment to start the node. (e.g. gpt-4o-mini)
And please prepare other model instances according to which functionalities you want to use (e.g. Embedding, Audio generation and so on)


And when you want to use [llama-cpp-python], run

```
roslaunch openai_ros openai.launch endpoint:=http://localhost:8000/v1 key:=dummy model:="<model name you used for server>"
```

How to use service
------------------

```
$ rosservice call /get_response '{prompt: "Write a poem about OpenAI"}'
```

```
$ rosservice call /get_embedding '{input: "Write a poem about OpenAI", model: "text-embedding-ada-002"}'
```

