# Configs
## Subscriber/Publisher Example
<pre><code>{
  "package_name": "sub",
  "package_info": {
    "version": "0.0.1",
    "description": "Description of the sub-package",
    "maintainer_mail": "test@test.de",
    "maintainer": "First Name",
    "license": "None yet",
    "test_depends": [
      "ament_copyright",
      "ament_flake8",
      "ament_pep257",
      "python3-pytest"
    ]
  },
  "subs": [
    {
      "topic": "exampletopic",
      "type": "",
      "callback": "-stdout-"
    }
  ],
  "pubs": [
    {
      "topic": "exampletopic",
      "type": "",
      "src": "-stdin-"
    }
  ],
  "additional_imports": [
    {
      "from_": "",
      "modules": []
    }
  ]
}</code></pre>
Type is one of the following:
- blabla
- test

