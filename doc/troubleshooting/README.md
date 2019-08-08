To generate the HTML:

pip3 install --user markdown pygments

# some help: python3 -m markdown --help
python3 -m markdown -x toc -x fenced_code -x codehilite troubleshooting.md > troubleshooting.html
pygmentize -S default -f html -a .codehilite > troubleshooting.css

Then surround the generated HTML file with 
<html>
  <head>
  <link rel="stylesheet" href="qrc:/css/troubleshooting.css"/>
  <head/>
  <body>
...
</html>
