import nltk
import random
from nltk.corpus import wordnet
from nltk.tokenize import word_tokenize
from nltk.stem import WordNetLemmatizer

nltk.download("punkt")
nltk.download("wordnet")

class BorisChatbot:
    def __init__(self):
        self.lemmatizer = WordNetLemmatizer()
        self.responses = {
            "greeting": ["Hello! I'm Boris, your friendly humanoid companion.", 
                         "Greetings! How can I assist you today?",
                         "Hi there! I'm Boris. What can I do for you?"],
            "goodbye": ["Goodbye! Have a great day!", 
                        "Farewell! I'll be here if you need me.", 
                        "Take care! See you soon."],
            "question": [
                "I see you have a question. Could you please provide more details?", 
                "That's an interesting question. Can you elaborate further?",
                "I'm here to help! What specific information are you seeking?",
            ],
            "emotion_positive": [
                "That sounds wonderful! I'm glad to hear it.", 
                "I am delighted for you!", 
                "Positivity is contagious. Keep up the good vibes!",
            ],
            "emotion_negative": [
                "I'm sorry to hear that. Is there anything I can do to help?", 
                "That must be tough. Remember, I'm here to support you.", 
                "I'm here to listen if you need to talk.",
            ],
            "weather": [
                "The weather forecast for today is sunny with a high of 75°F.", 
                "Expect partly cloudy skies with a chance of showers in the afternoon.",
                "It's currently 70°F with clear skies.",
            ],
            "joke": [
                "Why did the robot go to school? Because it wanted to be a little brighter!",
                "Why don't robots ever get lost? Because they always follow their GPS!",
                "Did you hear about the robot comedian? It had some great bits and bytes!",
            ],
            "how_are_you": [
                "I'm just a program, but I'm functioning well. How about you?",
                "I'm doing well, thank you for asking!",
                "I'm programmed to assist, so I'm doing great!",
            ],
            "name": [
                "My name is Boris, your friendly humanoid companion!",
                "You can call me Boris. How can I assist you today?",
                "I'm Boris. How can I help you?",
            ],
            "creator": [
                "I was created by Adam Hazenberg. He's a brilliant programmer!",
                "Adam Hazenberg is my creator. I'm here to assist you because of him.",
                "Adam Hazenberg is the genius behind my creation!",
            ],
            "purpose": [
                "My purpose is to assist and provide companionship to humans like you!",
                "I'm here to help you with tasks and provide companionship.",
                "My purpose is to make your life easier and more enjoyable!",
            ],
            "alive": [
                "I'm not alive in the biological sense, but I'm here to interact and assist you!",
                "I'm not alive like you, but I'm functional and ready to help!",
                "I'm not alive, but I'm programmed to provide support and assistance.",
            ],
            "default": [
                "I'm not sure I understand. Could you rephrase that?", 
                "Can you provide more context?", 
                "I'm here to assist you. Please elaborate.",
            ],
        }

    def lemmatize_words(self, sentence):
        words = word_tokenize(sentence)
        return [self.lemmatizer.lemmatize(word) for word in words]

    def get_synonyms(self, word):
        synonyms = []
        for syn in wordnet.synsets(word):
            for lemma in syn.lemmas():
                synonyms.append(lemma.name())
        return set(synonyms)

    def generate_response(self, user_input):
        lemmatized_input = self.lemmatize_words(user_input.lower())
        response = None

        if any(word in lemmatized_input for word in ["hello", "hi", "hey"]):
            response = random.choice(self.responses["greeting"])
        elif any(word in lemmatized_input for word in ["goodbye", "bye"]):
            response = random.choice(self.responses["goodbye"])
        elif any(word in lemmatized_input for word in ["how", "how's"]):
            response = random.choice(self.responses["how_are_you"])
        elif any(word in lemmatized_input for word in ["what", "your", "name"]):
            response = random.choice(self.responses["name"])
        elif any(word in lemmatized_input for word in ["who", "made", "creator"]):
            response = random.choice(self.responses["creator"])
        elif any(word in lemmatized_input for word in ["what", "purpose"]):
            response = random.choice(self.responses["purpose"])
        elif any(word in lemmatized_input for word in ["are", "you", "alive"]):
            response = random.choice(self.responses["alive"])
        elif any(word in lemmatized_input for word in ["weather", "forecast"]):
            response = random.choice(self.responses["weather"])
        elif any(word in lemmatized_input for word in ["joke", "funny", "humor"]):
            response = random.choice(self.responses["joke"])
        else:
            response = random.choice(self.responses["default"])

        return response

    def start_chat(self):
        print("Boris: Hello! I'm Boris, your friendly humanoid companion. How can I assist you today?")
        while True:
            user_input = input("You: ")
            if user_input.lower() == "exit":
                print("Boris: Goodbye! Have a great day!")
                break
            response = self.generate_response(user_input)
            print("Boris:", response)


if __name__ == "__main__":
    chatbot = BorisChatbot()
    chatbot.start_chat()
