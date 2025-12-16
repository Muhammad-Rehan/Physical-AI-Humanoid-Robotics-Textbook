import re
from typing import List, Dict

def chunk_text(text: str, max_chunk_size: int = 400, overlap: int = 50) -> List[Dict[str, str]]:
    """
    Splits text into logical chunks of approximately max_chunk_size words
    """
    # Split text into sentences
    sentences = re.split(r'[.!?]+\s+', text)

    chunks = []
    current_chunk = ""
    current_size = 0

    for sentence in sentences:
        # Estimate word count
        sentence_word_count = len(sentence.split())

        # If adding this sentence would exceed the max size
        if current_size + sentence_word_count > max_chunk_size and current_chunk:
            # Save the current chunk
            chunks.append({
                'text': current_chunk.strip(),
                'word_count': current_size
            })

            # Start a new chunk with overlap
            overlap_sentences = get_overlap_sentences(current_chunk, overlap)
            current_chunk = overlap_sentences + " " + sentence
            current_size = len(current_chunk.split())
        else:
            # Add sentence to current chunk
            current_chunk += " " + sentence
            current_size += sentence_word_count

    # Add the final chunk if it has content
    if current_chunk.strip():
        chunks.append({
            'text': current_chunk.strip(),
            'word_count': len(current_chunk.split())
        })

    return chunks

def get_overlap_sentences(text: str, word_count: int) -> str:
    """
    Gets the last few sentences from text with approximately word_count words
    """
    sentences = re.split(r'[.!?]+\s+', text)
    overlap_text = ""
    overlap_words = 0

    # Work backwards through sentences
    for sentence in reversed(sentences):
        sentence_words = len(sentence.split())
        if overlap_words + sentence_words <= word_count:
            overlap_text = sentence + " " + overlap_text
            overlap_words += sentence_words
        else:
            break

    return overlap_text.strip()

def chunk_by_paragraphs(text: str, max_chunk_size: int = 400) -> List[Dict[str, str]]:
    """
    Alternative method: chunks text by paragraphs when possible
    """
    # Split by paragraphs first
    paragraphs = text.split('\n\n')

    chunks = []
    current_chunk = ""
    current_size = 0

    for paragraph in paragraphs:
        paragraph_word_count = len(paragraph.split())

        if paragraph_word_count <= max_chunk_size:
            # If paragraph is small enough, add it as a chunk
            if paragraph_word_count > 0:
                chunks.append({
                    'text': paragraph.strip(),
                    'word_count': paragraph_word_count
                })
        else:
            # If paragraph is too large, split it using sentence method
            sub_chunks = chunk_text(paragraph, max_chunk_size)
            chunks.extend(sub_chunks)

    return chunks

if __name__ == "__main__":
    # Example usage
    sample_text = """
    This is a sample text for chunking. It contains multiple sentences.
    The algorithm should split this text into chunks of approximately the specified size.
    Each chunk should maintain logical coherence while respecting the size limits.
    Additional sentences make the text longer for testing purposes.
    """

    chunks = chunk_text(sample_text, max_chunk_size=10)
    print(f"Generated {len(chunks)} chunks:")
    for i, chunk in enumerate(chunks):
        print(f"\nChunk {i+1} ({chunk['word_count']} words):")
        print(chunk['text'])