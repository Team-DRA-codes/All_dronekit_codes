import requests

page = requests.get("https://sites.iitgn.ac.in/srip/results/")


page.status_code



from bs4 import BeautifulSoup
soup = BeautifulSoup(page.content, 'html.parser')
#print(soup.prettify())
print(soup.find_all(style="width: 216px"))