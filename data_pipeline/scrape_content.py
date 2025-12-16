import requests
from bs4 import BeautifulSoup
import os
from urllib.parse import urljoin, urlparse
import time
from dotenv import load_dotenv

def scrape_docusaurus_site(base_url):
    """
    Scrapes all text content from a Docusaurus site
    """
    all_text = []
    visited_urls = set()

    def get_page_content(url):
        try:
            response = requests.get(url)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Get text from main content areas
            content_selectors = [
                'main article',  # Common Docusaurus content selector
                '.markdown',     # Docusaurus markdown content
                '.theme-content', # Docusaurus theme content
                'article',       # Generic article
                '.container',    # Common container
            ]

            text_content = ""
            for selector in content_selectors:
                elements = soup.select(selector)
                if elements:
                    for element in elements:
                        text_content += element.get_text(separator=' ', strip=True) + "\n\n"
                    break

            # If no specific content found, get all body text
            if not text_content.strip():
                body = soup.find('body')
                if body:
                    text_content = body.get_text(separator=' ', strip=True)

            return text_content
        except Exception as e:
            print(f"Error scraping {url}: {e}")
            return ""

    def get_all_links(soup, base_url):
        links = []
        for link in soup.find_all('a', href=True):
            href = link['href']
            full_url = urljoin(base_url, href)

            # Only include internal links that are likely to be content pages
            if urlparse(full_url).netloc == urlparse(base_url).netloc:
                # Filter out common non-content URLs
                if not any(skip in full_url.lower() for skip in [
                    '/api/', '/tag/', '/category/', '.pdf', '.jpg', '.png', '.zip'
                ]):
                    links.append(full_url)
        return links

    # Start with the base URL
    urls_to_visit = [base_url]

    while urls_to_visit:
        current_url = urls_to_visit.pop(0)

        if current_url in visited_urls:
            continue

        print(f"Scraping: {current_url}")
        content = get_page_content(current_url)

        if content.strip():
            all_text.append({
                'url': current_url,
                'content': content,
                'title': extract_title(content)
            })

        visited_urls.add(current_url)

        # Get more URLs from the current page
        try:
            response = requests.get(current_url)
            soup = BeautifulSoup(response.content, 'html.parser')
            new_links = get_all_links(soup, base_url)

            for link in new_links:
                if link not in visited_urls and link not in urls_to_visit:
                    urls_to_visit.append(link)
        except Exception as e:
            print(f"Error getting links from {current_url}: {e}")

        # Be respectful to the server
        time.sleep(0.5)

    return all_text

def extract_title(content):
    """
    Extracts a title from content (first sentence or heading-like text)
    """
    lines = content.split('\n')
    for line in lines:
        line = line.strip()
        if line and len(line) < 100:  # Likely a title/heading
            return line
    return "Untitled Content"

if __name__ == "__main__":
    # Load environment variables from the parent directory
    load_dotenv(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'backend', '.env'))

    # Get base URL from environment or use a default
    base_url = os.getenv("DOCUSAURUS_BASE_URL", "https://your-docusaurus-site.com")

    print(f"Starting to scrape: {base_url}")
    scraped_content = scrape_docusaurus_site(base_url)

    print(f"Scraped {len(scraped_content)} pages")

    # Save to a file for further processing
    with open("scraped_content.txt", "w", encoding="utf-8") as f:
        for item in scraped_content:
            f.write(f"URL: {item['url']}\n")
            f.write(f"TITLE: {item['title']}\n")
            f.write(f"CONTENT: {item['content']}\n")
            f.write("-" * 80 + "\n")

    print("Content saved to scraped_content.txt")