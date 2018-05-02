/*
2018-02-09 TL First version, moved from kml-viewer.html

Description: General JS for kml-viewer.html

License: BSD 3-Clause
*/

// Inspiration: http://www.snip2code.com/Snippet/18734/
function toggle(curElement, elementId) {
    if (document.getElementById(curElement).innerHTML == 'show') {
        document.getElementById(curElement).innerHTML = 'hide';
    } else {
        document.getElementById(curElement).innerHTML = 'show';
    }
    var ele = document.getElementById(elementId);
    if(ele.style.display == "block") {
        ele.style.display = "none";
    } else {
        ele.style.display = "block";
    }
}
