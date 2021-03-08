import re

# List of available items on the fake menu
menu = [
    "apple",
    "apples",
    "banana",
    "bananas",
    "orange",
    "oranges"]


def check_if_any_word_in_menu(sentence):
    """
    Checks if a sentence contains any word in the menu. The
    function returns True if "sentence" contains things on the
    menu.

    Arguments
    ---------
        sentence : string
            A string to verify
    """
    # Removing punctuation and capital letters
    string = re.sub(r'[^\w\s]', '', sentence).lower()

    # Checking if there's an intersection
    co_occuring_words = set(menu) & set(string.split())

    return bool(co_occuring_words)


def extract_order(sentence):
    """
    Find words from the menu that are present in the sentence. The
    function returns a list containing everything from the menu that
    is present in the sentence.

    Arguments
    ---------
        sentence : string
            A string to check
    """
    # Removing punctuation and capital letters
    string = re.sub(r'[^\w\s]', '', sentence).lower()

    # Returning the intersection
    return list(set(menu) & set(string.split()))
