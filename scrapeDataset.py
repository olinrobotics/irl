import requests

def is_downloadable(url):
    """
    Does the url contain a downloadable resource?
    Base code from https://www.codementor.io/aviaryan/downloading-files-from-urls-in-python-77q3bs0un, modified Fall 2017 by L.Zuehsow

    >>> is_downloadable(2)
    False
    >>> is_downloadable('https://www.youtube.com/watch?v=9bZkp7q19f0')
    False
    >>> is_downloadable('http://google.com/favicon.ico')
    True
    >>> is_downloadable('http://image-net.org/nodes/13/03482001/27/273fe1f1dddd32d05a91584834c002ca6ba94336.thumb')
    True
    """

    h = requests.head(url, allow_redirects=True)
    header = h.headers
    content_type = header.get('content-type')
    content_length = header.get('content-length', None)
    print content_length

    if 'text' in content_type.lower():
        return False
    if 'html' in content_type.lower():
        return False
    if content_length and content_length < 2e8:  # 200 mb approx
        return False
    return True


if __name__ is "__main__":
    url = "http://image-net.org/nodes/13/03482001/27/273fe1f1dddd32d05a91584834c002ca6ba94336.thumb"
    if is_downloadable(url):
        r = requests.get(url, allow_redirects=True)
        filename = get_filename_from_cd(r.headers.get('content-disposition'))
        open(filename, 'wb').write(r.content)

# print is_downloadable('https://www.youtube.com/watch?v=9bZkp7q19f0')
# # >> False
# print is_downloadable('http://google.com/favicon.ico')
# # >> True
# print is_downloadable('http://image-net.org/nodes/13/03482001/27/273fe1f1dddd32d05a91584834c002ca6ba94336.thumb')
# # >> True