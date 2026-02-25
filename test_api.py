import requests
import os
from dotenv import load_dotenv

load_dotenv()
token = os.environ.get('GITHUB_TOKEN')
headers = {'Authorization': f'token {token}', 'Accept': 'application/vnd.github.v3+json'} if token else {'Accept': 'application/vnd.github.v3+json'}
print(f"Token loaded: {bool(token)}")

r = requests.get('https://api.github.com/search/code?q=rclpy+moveit+language:python&per_page=1', headers=headers)
data = r.json()
if 'items' in data and len(data['items']) > 0:
    repo = data['items'][0]['repository']
    print(repo.keys())
else:
    print(data)
