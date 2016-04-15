<?php
/**
 * Admin Add Interactive Marker View
 *
 * The add interactive marker view allows an admin to add a new interactive marker setting to the database.
 *
 * @author		Russell Toris - rctoris@wpi.edu
 * @copyright	2014 Worcester Polytechnic Institute
 * @link		https://github.com/WPI-RAIL/rms
 * @since		RMS v 2.0.0
 * @version		2.0.9
 * @package		app.View.Ims
 */
?>

<header class="special container">
	<span class="icon fa-pencil"></span>
	<h2>Add Interactive Marker Setting</h2>
</header>

<?php echo $this->element('im_form'); ?>
