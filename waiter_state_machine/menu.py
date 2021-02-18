import re

menu = ["apple", "apples", "banana", "bananas", "orange", "oranges"]

def check_if_any_word_in_menu(sentence):
    
    # Remove punctuation and capital letters
    string = re.sub(r'[^\w\s]','',sentence).lower()

    # Checking if there's an intersection
    co_occuring_words = set(menu) & set(string.split())

    return bool(co_occuring_words)


def extract_order(sentence):

    # Remove punctuation and capital letters
    string = re.sub(r'[^\w\s]','',sentence).lower()

    # Returning the intersection
    return (set(menu) & set(string.split()))