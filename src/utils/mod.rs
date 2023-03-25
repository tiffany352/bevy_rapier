use bevy::prelude::Transform;
use rapier::math::{Isometry, Real};

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim2")]
pub fn iso_to_transform(
    iso: &Isometry<Real>,
    physics_scale: Real,
    physics_transform: &Isometry<Real>,
) -> Transform {
    let mut iso = iso.clone();
    iso.translation.vector *= physics_scale;
    let iso = physics_transform * iso;
    Transform {
        translation: (iso.translation.vector.push(0.0)).into(),
        rotation: bevy::prelude::Quat::from_rotation_z(iso.rotation.angle()),
        ..Default::default()
    }
}

/// Converts a Rapier isometry to a Bevy transform.
///
/// The translation is multiplied by the `physics_scale`.
#[cfg(feature = "dim3")]
pub fn iso_to_transform(
    iso: &Isometry<Real>,
    physics_scale: Real,
    physics_transform: &Isometry<Real>,
) -> Transform {
    let mut iso = iso.clone();
    iso.translation.vector *= physics_scale;
    let iso = physics_transform * iso;
    Transform {
        translation: (iso.translation.vector * physics_scale).into(),
        rotation: iso.rotation.into(),
        ..Default::default()
    }
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim2")]
pub(crate) fn transform_to_iso(
    transform: &Transform,
    physics_scale: Real,
    physics_transform: &Isometry<Real>,
) -> Isometry<Real> {
    use bevy::math::Vec3Swizzles;
    physics_transform.inverse()
        * Isometry::new(
            (transform.translation / physics_scale).xy().into(),
            transform.rotation.to_scaled_axis().z,
        )
}

/// Converts a Bevy transform to a Rapier isometry.
///
/// The translation is divided by the `physics_scale`.
#[cfg(feature = "dim3")]
pub(crate) fn transform_to_iso(
    transform: &Transform,
    physics_scale: Real,
    physics_transform: &Isometry<Real>,
) -> Isometry<Real> {
    physics_transform.inverse()
        * Isometry::from_parts(
            (transform.translation / physics_scale).into(),
            transform.rotation.into(),
        )
}

#[cfg(test)]
#[cfg(feature = "dim3")]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use bevy::prelude::Transform;
    use glam::EulerRot;
    use na::{vector, Rotation, Translation, Unit};

    #[test]
    fn convert_back_to_equal_transform() {
        let transform = Transform {
            translation: bevy::prelude::Vec3::new(-2.1855694e-8, 0.0, 0.0),
            rotation: bevy::prelude::Quat::from_xyzw(0.99999994, 0.0, 1.6292068e-7, 0.0)
                .normalize(),
            ..Default::default()
        };
        let converted_transform = iso_to_transform(
            &transform_to_iso(&transform, 1.0, &Isometry::identity()),
            1.0,
            &Isometry::identity(),
        );
        assert_eq!(converted_transform, transform);
    }

    #[test]
    fn convert_back_to_transformed() {
        let transform = Transform {
            translation: bevy::prelude::Vec3::new(3.0, 2.0, 1.0),
            rotation: bevy::prelude::Quat::from_euler(EulerRot::XYZ, 5.0, 4.0, 3.0),
            ..Default::default()
        };
        let iso = Isometry::from_parts(
            Translation::from(vector![10.0, 20.0, 30.0]),
            Unit::from(Rotation::from_euler_angles(45.0, 45.0, 45.0)),
        );
        let converted_transform =
            iso_to_transform(&transform_to_iso(&transform, 1.0, &iso), 1.0, &iso);
        assert_abs_diff_eq!(
            converted_transform.translation,
            transform.translation,
            epsilon = 1e-3
        );
        assert_abs_diff_eq!(
            converted_transform.rotation,
            transform.rotation,
            epsilon = 1e-3
        );
        assert_abs_diff_eq!(converted_transform.scale, transform.scale, epsilon = 1e-3);
    }
}
