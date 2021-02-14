<%@taglib uri='http://java.sun.com/jsp/jstl/core' prefix='c'%>
<!DOCTYPE html>
<html>
    <head>
        <title>Cheesy Path</title>
        
        <script src='https://ajax.googleapis.com/ajax/libs/jquery/3.1.1/jquery.min.js'></script>
        <script src='https://ajax.googleapis.com/ajax/libs/jqueryui/1.12.1/jquery-ui.min.js'></script>
        <script type='text/javascript' src='<c:url value='/resources/js/script.js' />'></script>

        <link rel='shortcut icon' href='https://media.team254.com/homepage/icons/favicon32.png' />
        <link href='<c:url value='/resources/css/style.css' />' rel='stylesheet'>
    </head>
    <body onload='init()'>
        <input id='title' placeholder='Title'>
        <div id='canvases'>
            <canvas id='background'></canvas>
            <canvas id='field'></canvas>
        </div>
        <div class='buttonContainer'>
            <button onclick='addPoint()'>Add Point</button>
            <button onclick='update()'>Update</button>
            <button onclick='draw(3)'>Animate</button>
            <button onclick="showWaypointsList()">Waypoints Code</button>
            <select onchange='changeField(this.value)'>
                <option value="6_field1" selected>6_field1</option>
                <option value="6_field2">6_field2</option>
                <option value="7_field1">7_field1</option>
                <option value="7_field2">7_field2</option>
                <option value="7_field3">7_field3</option>
            </select>
            <span class='checkbox'>Is reversed: <input type='checkbox' id='isReversed'></span>
        </div>
        <table>
            <thead>
                <th></th>
                <th>X</th>
                <th>Y</th>
                <th>Heading</th>
                <th>Comments</th>
                <th>Enabled</th>
                <th>Delete</th>
            </thead>
            <tbody>
                <tr>
                    <td class='drag-handler'>&#x205e;</td>
                    <td class='x'><input type='number' value='0'></td>
                    <td class='y'><input type='number' value='0'></td>
                    <td class='heading'><input type='number' value='0'></td>
                    <td class='comments'><input type='search' placeholder='Comments'></td>
                    <td class='enabled'><input type='checkbox' checked></td>
                    <td class='delete'><button onclick='$(this).parent().parent().remove();update()'>&times;</button></td>
                </tr>
            </tbody>
        </table>
        <dialog id="waypointsDialog">
            <button onclick='this.parentElement.close()' class="close-button">&times;</button>
            <h3>Waypoints List</h3>
            <pre><code id="waypointsOutput"></code></pre>
        </dialog>

        <input type='file' id='upl' style='display:none;'>
    </body>
</html>

<script>
    $('table tbody').sortable({
        helper: fixWidthHelper,
        deactivate: update
    }).disableSelection();
        
    function fixWidthHelper(e, ui) {
        ui.children().each(function() {
            $(this).width($(this).width());
        });
        return ui;
    }
</script>
