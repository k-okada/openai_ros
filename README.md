openai_ros
==========

ROS1 wrapper for (OpenAI Python Library)[https://pypi.org/project/openai/].

How to start program
--------------------

Start from sample lamuch file
```
$ export OPENAI_API_KEY="xxxxxxxxx"
$ roslaunch openai_ros openai.launch max_tokens:=256 model:=gpt-3.5-turbo
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
roslaunch openai_ros openai.launch backend:=azure key:=<api key> model:=<deployed chat model instance name> azure_endpoint:=<endpoint url> 
```

When you use Azure OpenAI, firstly please prepare some chat or completion model instance deployment to start the node.
And please prepare other model instances according to which functionalities you want to use (e.g. Embedding, Audio generation and so on)

How to use service
------------------

```
$ rosservice call /get_response '{prompt: "Write a poem about OpenAI"}'
```

```
$ rosservice call /get_embedding '{input: "Write a poem about OpenAI", model: "text-embedding-ada-002"}'
```

