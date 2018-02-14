# Making requests using the Python requests library
Check Python version, use later.
```sh
$ python -V
```

## requests installation
The following upgrade and packages might be needed under Python 2.7
Ensure that requests is installed but note that for Python 3 the extra security options can just be installed whereas Python 2.7 should follow a more manual installation; NOTE that this is only necessary when using SSL.
Python <2.7.9
```sh
$ pip install --upgrade setuptools
$ apt install python-dev libssl-dev libffi-dev
$ pip install requests
$ pip install pyOpenSSL
$ pip install cryptography
$ pip install idna
$ pip install --upgrade ndg-httpsclient
```
Python >2.7.9
```sh
$ pip install requests
$ pip install --upgrade requests[security]
```

Verification can also be bypassed on a HTTPs request using the `verify=True` parameter in the python request.

References:
[urllib3](https://urllib3.readthedocs.io/en/latest/advanced-usage.html#ssl-warnings)
[stackoverflow: SSL InsecurePlatform](https://stackoverflow.com/questions/29099404/ssl-insecureplatform-error-when-using-requests-package)
[stackoverflow: requests differences](https://stackoverflow.com/questions/31811949/pip-install-requestssecurity-vs-pip-install-requests-difference)
[github: mismatch between hostnames](https://github.com/jakubroztocil/httpie/issues/262)
[Certifi: Trust Database for Humans](http://certifiio.readthedocs.io/en/latest/)
