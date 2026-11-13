use fem_types::{ElemId, MaterialAssignment, MaterialAssignmentError, MaterialId};

#[test]
fn uniform_assignment_returns_the_same_material_for_every_element() {
    let assignment = MaterialAssignment::Uniform(MaterialId(2));

    assert!(assignment.is_uniform());
    for index in [0, 1, 100] {
        assert_eq!(assignment.get(ElemId(index)), Some(MaterialId(2)));
    }
    assert_eq!(assignment.material_count(), 1);
    assignment.validate(3, 3).unwrap();
}

#[test]
fn per_element_assignment_requires_one_entry_per_element() {
    let assignment =
        MaterialAssignment::PerElement(vec![MaterialId(0), MaterialId(1), MaterialId(0)]);

    assert!(!assignment.is_uniform());
    assert_eq!(assignment.material_count(), 3);
    assert_eq!(assignment.get(ElemId(1)), Some(MaterialId(1)));
    assert_eq!(assignment.get(ElemId(3)), None);
    assignment.validate(3, 2).unwrap();

    assert_eq!(
        assignment.validate(2, 2).unwrap_err(),
        MaterialAssignmentError::Length {
            expected: 2,
            found: 3,
        }
    );
}

#[test]
fn rejects_unknown_material_ids() {
    assert_eq!(
        MaterialAssignment::Uniform(MaterialId(1))
            .validate(1, 1)
            .unwrap_err(),
        MaterialAssignmentError::UnknownMaterial {
            element: None,
            material: MaterialId(1),
        }
    );

    assert_eq!(
        MaterialAssignment::PerElement(vec![MaterialId(0), MaterialId(2)])
            .validate(2, 2)
            .unwrap_err(),
        MaterialAssignmentError::UnknownMaterial {
            element: Some(1),
            material: MaterialId(2),
        }
    );
}

#[test]
fn assignment_expands_lazily_and_compacts_again() {
    let mut assignment = MaterialAssignment::Uniform(MaterialId(0));

    // Assigning the same material keeps the uniform representation.
    assignment.set(ElemId(1), MaterialId(0), 3);
    assert!(assignment.is_uniform());

    assignment.set(ElemId(1), MaterialId(1), 3);
    assert_eq!(
        assignment,
        MaterialAssignment::PerElement(vec![MaterialId(0), MaterialId(1), MaterialId(0)])
    );

    assignment.set(ElemId(0), MaterialId(1), 3);
    assignment.set(ElemId(2), MaterialId(1), 3);
    assignment.compact();
    assert_eq!(assignment, MaterialAssignment::Uniform(MaterialId(1)));

    assignment.set_all(MaterialId(0));
    assert_eq!(assignment, MaterialAssignment::Uniform(MaterialId(0)));
}
