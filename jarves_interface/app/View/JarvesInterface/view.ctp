<!-- this auto-generates the connecting to ROS code -->
<?php echo $this->Rms->ros($environment['Rosbridge']['uri']); ?>


<script>
        var topic = new ROSLIB.Topic({
                ros : _ROS,
                name : '/echo',
                messageType : 'std_msgs/String'
                });

        var send_location = new ROSLIB.Service({
                ros : _ROS,
                name : '/office_destination',
                serviceType : 'jarves/Destination'
                });
</script>

<header class="special container">
        <span class="icon fa-android" aria-hidden="true"></span>
        <h2>Jarves</h2>
</header>

<!-- we need 'n' buttons to signify the office locations we have in our database -->

<section class="wrapper style4 container">
        <div class="content center">
                <button id="send" class="button">Dr H Christensen</button>
                <script>
                        $('#send').click(function() {
                                var msg = new ROSLIB.Message({
                                        data : $('#message').val()
                                });
                                topic.publish(msg);
                        });
                </script>
                
                <br />
                <br />
                <button id="send" class="button">Dr S Chernova</button>
                <script>
                        $('#send').click(function() {
                                var msg = new ROSLIB.Message({
                                        data : $('#message').val()
                                });
                                topic.publish(msg);
                        });
                </script>
        </div>
</section>
