# from openai import OpenAI
import requests

class OllamaWrapper:
  def __init__(self, model_name, model_host, api_key):
    self.model = model_name
    self.llm_host = model_host
    # self.llm_client = OpenAI(
    #     base_url=model_host + '/v1',
    #     api_key=api_key,
    # )

  def run(self,prompt):
    headers = {}
    if self.api_key:
        headers['Authorization'] = f"Bearer {self.api_key}"
    response = requests.post(
        f'{self.llm_host}/v1/chat/completions',
        json={
            'model': self.llm_model,
            'messages': [
                {
                    'role': 'system',
                    'content': prompt['system'],
                },
                {
                    'role': 'user',
                    'content': prompt['user'],
                }],
            'temperature': 0.0,
            'stream': False},
        headers=headers)
    if response.status_code != requests.codes.ok:
        raise RuntimeError(
            f'Ollama server response [{response.status_code}]: {response.text}')
    response_json = response.json()['choices'][0]

    explanation = str(response_json['message']['content']).strip()
    return explanation

    # response = self.llm_client.chat.completions.create(
    #   model=self.model,
    #   temperature=0.0,
    #   messages=[
    #     {
    #       'role': 'system',
    #       'content': prompt['system'],
    #     },
    #     {
    #       'role': 'user',
    #       'content': prompt['user'],
    #     }
    #   ])
    # return response.choices[0].message.content

if __name__=='__main__':
  wrapper = OllamaWrapper(model_name='phi4:latest',model_host='http://143.225.85.134:11434',api_key='')
  res=wrapper.run(prompt={
  'system': 'You are a helpful assistant',
  'user': 'Does pineapple belong on Neapolitan pizza?'
  })
  print(res)