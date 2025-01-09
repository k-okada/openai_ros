#!/usr/bin/env python3

import base64
import json

import rospy
from openai import AzureOpenAI, OpenAI
from typing import Optional

from openai_ros.msg import StringArray
from openai_ros.srv import (AudioSpeech, AudioSpeechResponse, ChatCompletions,
                            ChatCompletionsResponse, Completion,
                            CompletionResponse, Embedding, EmbeddingResponse)


def legacy_servicer(req):
    global client, max_tokens, model
    res = CompletionResponse()

    response = client.completions.create(model=model, prompt=req.prompt, temperature=req.temperature, max_tokens=max_tokens)

    res.finish_reason = response.choices[0].finish_reason
    res.text = response.choices[0].text
    res.model = response.model
    res.completion_tokens = response.usage.completion_tokens
    res.prompt_tokens = response.usage.prompt_tokens
    res.total_tokens = response.usage.total_tokens

    # When response is not working, completion_tokens is None, which chase error on CompleteResponse format(int32)
    if not isinstance(res.completion_tokens, int):
        res.completion_tokens = -1
    return res

def chat_servicer(req):
    global client, max_tokens, model
    res = CompletionResponse()

    response = client.chat.completions.create(model=model, messages=[{"role": "user", "content": req.prompt}], temperature=req.temperature, max_tokens=max_tokens)

    res.finish_reason = response.choices[0].finish_reason
    res.text = response.choices[0].message.content
    res.model = response.model
    res.completion_tokens = response.usage.completion_tokens
    res.prompt_tokens = response.usage.prompt_tokens
    res.total_tokens = response.usage.total_tokens

    # When response is not working, completion_tokens is None, which chase error on CompleteResponse format(int32)
    if not isinstance(res.completion_tokens, int):
        res.completion_tokens = -1
    return res

def chat_completions_servicer(req):
    global client
    res = ChatCompletionsResponse()

    kwargs = dict(model=req.model, messages=json.loads(req.messages), temperature=req.temperature, frequency_penalty=req.frequency_penalty)
    if req.max_tokens > 0:
        kwargs['max_tokens'] = req.max_tokens
    response = client.chat.completions.create(**kwargs)

    res.finish_reason = response.choices[0].finish_reason
    res.content = response.choices[0].message.content
    res.model = response.model
    res.completion_tokens = response.usage.completion_tokens
    res.prompt_tokens = response.usage.prompt_tokens
    res.total_tokens = response.usage.total_tokens

    # When response is not working, completion_tokens is None, which chase error on CompleteResponse format(int32)
    if not isinstance(res.completion_tokens, int):
        res.completion_tokens = -1
    return res

def audio_speech_servicer(req):
    global client
    res = AudioSpeechResponse()

    kwargs = dict(model=req.model, input=req.input, voice=req.voice)
    if len(req.response_format) > 0:
        kwargs['response_format'] = req.response_format
    response = client.audio.speech.create(**kwargs)

    res.content = base64.b64encode(response.content)
    return res

def embedding_servicer(req):
    global client
    res = EmbeddingResponse()
    response = client.embeddings.create(
        input=[req.input],
        model=req.model,
    )
    res.embedding = response.data[0].embedding
    res.model = response.model
    res.prompt_tokens = response.usage.prompt_tokens
    res.total_tokens = response.usage.total_tokens
    return res


def main():
    global client, max_tokens, model
    pub = rospy.Publisher('available_models', StringArray, queue_size=1, latch=True)
    rospy.init_node('openai_node', anonymous=True)
    endpoint: Optional[str] = rospy.get_param("~endpoint", None)
    backend: Optional[str] = rospy.get_param("~backend", None) # openai or azure or None.

    if backend is not None:
        rospy.logdebug("Backend is set to " + backend)
    elif (endpoint is not None and len(endpoint) != 0) and endpoint.split("/")[2].endswith('azure.com'):
        backend = "azure"
    else:
        backend = "openai"

    rospy.loginfo("Using " + backend + " backend")

    if backend == "openai":
        if endpoint is None or len(endpoint) == 0:
            client = OpenAI(api_key=rospy.get_param('~key'))
        else:
            client = OpenAI(
                api_key=rospy.get_param('~key'),
                base_url=endpoint,
            )
    elif backend == "azure":
        client = AzureOpenAI(
            api_key=rospy.get_param("~key"),
            azure_endpoint=endpoint,
            api_version=rospy.get_param("~azure_api_version", "2024-07-01-preview"),
        )
    else:
        rospy.logerr("Invalid backend: " + backend)
        return

    max_tokens = rospy.get_param('~max_tokens', default=256)
    model = rospy.get_param('~model', default='gpt-3.5-turbo')

    models_msg = StringArray()
    for m in client.models.list():
        models_msg.data.append(m.id)

    if model not in models_msg.data:
        rospy.logerr(model + " is not an available model")
        rospy.logerr(models_msg.data)
        return

    pub.publish(models_msg)

    endpoint = "chat"
    try:
        client.chat.completions.create(model=model, messages=[{"role": "user", "content": "ignore this message"}], max_tokens=1)
    except:
        endpoint = "legacy"

    rospy.loginfo("Using " + model + " though " + endpoint + " endpoint")

    if endpoint == "legacy":
        rospy.Service('get_response', Completion, legacy_servicer)
    else:
        rospy.Service('get_response', Completion, chat_servicer)

    if endpoint == "chat":
        rospy.Service('chat_completions', ChatCompletions, chat_completions_servicer)
        rospy.Service('audio_speech', AudioSpeech, audio_speech_servicer)

    rospy.Service("get_embedding", Embedding, embedding_servicer)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
