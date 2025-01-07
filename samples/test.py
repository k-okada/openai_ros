#!/usr/bin/env python

import argparse
import rospy
import json
import base64
from openai_ros.srv import ChatCompletions, ChatCompletionsRequest
from openai_ros.srv import AudioSpeech, AudioSpeechRequest
from openai_ros.srv import Embedding, EmbeddingRequest

##
def api_reference_making_requests(model):
    global chat_completion, audio_speech, get_embedding
    rospy.loginfo("-- Example from https://platform.openai.com/docs/api-reference/making-requests")
    req = ChatCompletionsRequest(model = model,
                                 messages = '[{"role": "user", "content": "Say this is a test!"}]', 
                                 temperature = 0.7)
    rospy.loginfo("{}".format(req.messages))
    ret = chat_completion(req)
    rospy.loginfo(">> {}".format(ret.content))


##
def api_reference_audio(model):
    global chat_completion, audio_speech, get_embedding
    req = AudioSpeechRequest(model = model,
                             input = 'The quick brown fox jumped over the lazy dog.',
                             voice = 'alloy')
    rospy.loginfo("-- Example from https://platform.openai.com/docs/api-reference/audio")
    ret = audio_speech(req)
    with open ('audio_speech.mp3', 'wb') as f:
        f.write(base64.b64decode(ret.content))
    rospy.loginfo(">> write to audio_speech.mp3")

##
def api_reference_embedding(model):
    global chat_completion, audio_speech, get_embedding
    req = EmbeddingRequest(model = model,
                           input = 'The food was delicious and the waiter...')
    rospy.loginfo("-- Example from https://platform.openai.com/docs/api-reference/embeddings")
    ret = get_embedding(req)
    rospy.loginfo("Result: {}".format(ret))

##
def api_reference_chat_create(model):
    global chat_completion, audio_speech, get_embedding
    rospy.loginfo("-- Example from https://platform.openai.com/docs/api-reference/chat/create")
    req = ChatCompletionsRequest(model = model,
                                 messages = '[{"role": "system", "content": "You are a helpful assistant."}, {"role": "user", "content": "Hello!"}]',
                                 temperature = 1.0)
    rospy.loginfo("{}".format(req.messages))
    ret = chat_completion(req)
    rospy.loginfo(">> {}".format(ret.content))

##
def guides_vision(model):
    global chat_completion, audio_speech, get_embedding
    rospy.loginfo("-- Example from https://platform.openai.com/docs/guides/vision/quick-start")
    req = ChatCompletionsRequest(model = model,
                                 messages = json.dumps([{"role": "user", "content": [ {"type": "text", "text": "What's in this image?"}, {"type": "image_url", "image_url" : {"url": "https://upload.wikimedia.org/wikipedia/commons/thumb/d/dd/Gfp-wisconsin-madison-the-nature-boardwalk.jpg/2560px-Gfp-wisconsin-madison-the-nature-boardwalk.jpg"}}]}]),
                                 temperature = 1.0)
    rospy.loginfo("{}".format(req.messages))
    ret = chat_completion(req)
    rospy.loginfo(">> {}".format(ret.content))

    ##
    import cv2
    img = cv2.imread('/usr/share/backgrounds/ryan-stone-skykomish-river.jpg')
    _, buf = cv2.imencode('.jpg', img)
    req = ChatCompletionsRequest(model = model,
                                 messages = json.dumps([{"role": "user", "content": [ {"type": "text", "text": "What's in this image?"}, {"type": "image_url", "image_url" : {"url": "data:image/jpeg;base64,{}".format(base64.b64encode(buf).decode('utf-8'))}}]}]),
                                 temperature = 1.0, max_tokens=300)
    rospy.loginfo("{}".format(req.messages[0:255]))
    ret = chat_completion(req)
    rospy.loginfo(">> {}".format(ret.content))


def main(mode,
         model_embedding,
         model_chat,
         model_chat_vision,
         model_audio_speech):
    global chat_completion, audio_speech, get_embedding
    ## 
    rospy.init_node('say_this_is_a_test')

    if mode == "chat":
        rospy.wait_for_service('/chat_completions')
        chat_completion = rospy.ServiceProxy('/chat_completions', ChatCompletions)
        rospy.wait_for_service('/audio_speech')
        audio_speech = rospy.ServiceProxy('/audio_speech', AudioSpeech)

    rospy.wait_for_service("/get_embedding")
    get_embedding = rospy.ServiceProxy('/get_embedding', Embedding)

    if mode == "chat":
        api_reference_making_requests(model=model_chat)
        api_reference_audio(model_audio_speech)
        api_reference_chat_create(model=model_chat)
        guides_vision(model=model_chat_vision)
    api_reference_embedding(model=model_embedding)

'''
from openai import OpenAI
client = OpenAI()

# completion = client.chat.completions.create(
#   model="gpt-3.5-turbo",
#   messages=[
#     {"role": "system", "content": "You are a poetic assistant, skilled in explaining complex programming concepts with creative flair."},
#     {"role": "user", "content": "Compose a poem that explains the concept of recursion in programming."}
#   ]
# )

# completion = client.chat.completions.create(
#     model="gpt-3.5-turbo",
#     temperature=0.7,
#     messages=[{"role": "user", "content": "Say this is a test!"}],
# )

print(completion.choices[0].message)
'''

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="chat", choices=["chat", "legacy"])
    parser.add_argument("--model-embedding", default="text-embedding-ada-002")
    parser.add_argument("--model-chat", default="gpt-3.5-turbo")
    parser.add_argument("--model-chat-vision", default="gpt-4o-mini")
    parser.add_argument("--model-audio-speech", default="tts-1")
    args = parser.parse_args()
    main(
        mode=args.mode,
        model_embedding=args.model_embedding,
        model_chat=args.model_chat,
        model_chat_vision=args.model_chat_vision,
        model_audio_speech=args.model_audio_speech,
        )
