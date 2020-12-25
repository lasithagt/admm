template <typename TRAIT>
iit::Kuka::dyn::tpl::InertiaProperties<TRAIT>::InertiaProperties()
{
    com_link1 = iit::rbd::Vector3d(0.0,-0.03,0.12).cast<Scalar>();
    tensor_link1.fill(
        Scalar(4.0),
        com_link1,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.1612),
                Scalar(0.1476),
                Scalar(0.0236),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(-0.0144)) );

    com_link2 = iit::rbd::Vector3d(3.0E-4,0.059,0.042).cast<Scalar>();
    tensor_link2.fill(
        Scalar(4.0),
        com_link2,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.071),
                Scalar(0.025),
                Scalar(0.0579),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_link3 = iit::rbd::Vector3d(0.0,0.03,0.13).cast<Scalar>();
    tensor_link3.fill(
        Scalar(3.0),
        com_link3,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.1334),
                Scalar(0.1257),
                Scalar(0.0127),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0117)) );

    com_link4 = iit::rbd::Vector3d(0.0,0.067,0.034).cast<Scalar>();
    tensor_link4.fill(
        Scalar(2.7),
        com_link4,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0452),
                Scalar(0.0131),
                Scalar(0.0411),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0061)) );

    com_link5 = iit::rbd::Vector3d(1.0E-4,0.021,0.076).cast<Scalar>();
    tensor_link5.fill(
        Scalar(1.7),
        com_link5,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.0306),
                Scalar(0.0279),
                Scalar(0.0058),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0027)) );

    com_link6 = iit::rbd::Vector3d(0.0,6.0E-4,4.0E-4).cast<Scalar>();
    tensor_link6.fill(
        Scalar(1.8),
        com_link6,
        rbd::Utils::buildInertiaTensor(
                Scalar(5.0E-4),
                Scalar(0.0036),
                Scalar(0.0047),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

    com_link7 = iit::rbd::Vector3d(0.0,0.0,0.02).cast<Scalar>();
    tensor_link7.fill(
        Scalar(0.3),
        com_link7,
        rbd::Utils::buildInertiaTensor(
                Scalar(0.001),
                Scalar(0.001),
                Scalar(0.001),
                Scalar(0.0),
                Scalar(0.0),
                Scalar(0.0)) );

}

