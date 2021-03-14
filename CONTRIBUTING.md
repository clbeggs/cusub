
# Table of Contents


- [Table of Contents](#table-of-contents)
    - [Writing documentation](#writing-documentation)
      - [Building documentation](#building-documentation)
 



## Writing documentation

Use [Google style](http://sphinxcontrib-napoleon.readthedocs.io/en/latest/example_google.html)
docstrings.

Example:

**Note:** The `r` prefix of the docstring is necessary, and denotes a [raw string](https://docs.python.org/2/reference/lexical_analysis.html#string-literals).
```python

def foo_bar(arg1='default1', arg2='default2'):
    r"""Description of function

        Longer description of the function.
        Can be extended over multiple lines.

        Katex Inline math example: :math:`f(x) = x^2 + \sin x`

        Katex math block example:

        .. math::
            g(x) = \cos x^2 - 1


        Args:
            arg1 (type): Short description of arg1
            arg2 (type): Short description of arg2

        Returns:
            type: Short description of return value 1
            type: Short description of return value 2 

        Inputs:
            - arg1: <Optional>Long description of arg1
            - arg2: <Optional>Long description of arg2

        Outputs:
            - <Optional>Long description of return value 1
            - <Optional>Long description of return value 2

     """
```



### Building documentation

To build to documentation:

1. Build Robosub with catkin

2. Install prerequisites:

```bash
cd cusub_documentation
pip install -r requirements.txt

# Must install katex:
npm install -g katex
```

3. Generate Docs, generated files will be in `cusub_documentation/build/html`

```bash
make html
```

